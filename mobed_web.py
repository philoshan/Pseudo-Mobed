"""
MOBED 웹 조종 서버 (PC / 폰 브라우저)

사용법:
    python mobed_web.py COM9              # 블루투스 COM 포트 (또는 USB 시리얼)
    python mobed_web.py SIM               # 로봇 없이 화면만 시험 (명령을 터미널에 출력)
    python mobed_web.py COM9 --local      # 이 PC에서만 접속 허용
    python mobed_web.py COM9 --http-port 8080

실행하면 접속 주소가 출력된다. PC는 http://localhost:8000, 폰은 같은 와이파이에서 출력된 LAN 주소로 접속.
설치: pip install pyserial aiohttp

안전장치
  - 브라우저 입력이 0.3초 이상 끊기거나 접속이 끊기면 즉시 정지
  - 로봇 쪽에도 1초 명령 타임아웃이 따로 있음
"""
import argparse
import asyncio
import json
import re
import socket
import threading
import time
from pathlib import Path

import serial
from aiohttp import WSMsgType, web

WEB_DIR = Path(__file__).parent / "web"
BAUD = 115200
SEND_HZ = 20           # 로봇으로 D 명령 전송 주기
INPUT_TIMEOUT = 0.3    # 이 시간 동안 브라우저 입력이 없으면 정지
SPIN_MAX = 40          # 제자리 회전 시 최대 좌우 속도 (%)
STATUS_PERIOD = 0.5    # '?' 조회 주기
NO_REPLY_REOPEN = 6.0  # 이 시간 동안 로봇 응답이 없으면 포트 재연결
KV = re.compile(r"(\w+)=(-?\d+(?:\.\d+)?)")


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class Robot(threading.Thread):
    """시리얼 포트를 쥐고 20Hz로 주행 명령을 보내고, 상태를 읽어 둔다."""

    def __init__(self, port, sim=False):
        super().__init__(daemon=True)
        self.port = port
        self.sim = sim
        self.lock = threading.Lock()
        self.speed = 0
        self.steer = 0
        self.spin = False
        self.last_input = 0.0
        self.pending = []
        self.status = {}
        self.status_time = 0.0
        self.open_time = 0.0
        self.connected = False
        self.ser = None
        self.buf = b""
        self.running = True
        self._last_printed = None

    # ---- 서버(비동기) 쪽에서 부르는 함수들 ----
    def set_drive(self, speed, steer, spin):
        with self.lock:
            self.speed, self.steer, self.spin = speed, steer, spin
            self.last_input = time.time()

    def stop(self):
        with self.lock:
            self.speed = self.steer = 0
            self.spin = False
            self.pending.append("X")

    def queue(self, cmd):
        with self.lock:
            self.pending.append(cmd)

    def snapshot(self):
        with self.lock:
            st = dict(self.status)
            age = time.time() - self.status_time if self.status_time else None
            return {"link": self.connected, "age": age, "status": st}

    # ---- 시리얼 ----
    def _ensure_port(self):
        if self.sim:
            self.connected = True
            return True
        if self.ser and self.ser.is_open:
            return True
        try:
            self.ser = serial.Serial(self.port, BAUD, timeout=0, write_timeout=0.2)
            time.sleep(2)  # 포트 열릴 때 보드 리셋 대기
            self.open_time = time.time()
            self.connected = True
            print(f"[serial] {self.port} 연결됨")
            return True
        except (serial.SerialException, OSError) as e:
            if self.connected or self.ser is None:
                print(f"[serial] {self.port} 열기 실패: {e}")
            self.connected = False
            self.ser = None
            return False

    def _close(self):
        self.connected = False
        try:
            if self.ser:
                self.ser.close()
        except Exception:
            pass
        self.ser = None

    def _send(self, text):
        if self.sim:
            if text != self._last_printed:
                print(f"[sim] {text}")
                self._last_printed = text
            return
        try:
            self.ser.write((text + "\n").encode())
        except serial.SerialTimeoutException:
            pass  # 블루투스 송신이 막히면 이번 명령은 건너뜀

    def _read(self):
        if self.sim:
            with self.lock:
                self.status = {"rpmL": 0.0, "rpmR": 0.0, "pitch": 0.0, "roll": 0.0, "level": 0, "imu": 1}
                self.status_time = time.time()
            return
        self.buf += self.ser.read(512)
        while b"\n" in self.buf:
            line, self.buf = self.buf.split(b"\n", 1)
            text = line.decode(errors="ignore").strip()
            if text.startswith("S "):
                vals = {k: float(v) for k, v in KV.findall(text)}
                if "rpmL" in vals:
                    with self.lock:
                        self.status = vals
                        self.status_time = time.time()
        if len(self.buf) > 2048:
            self.buf = b""

    def run(self):
        period = 1.0 / SEND_HZ
        next_t = time.time()
        last_q = 0.0
        while self.running:
            if not self._ensure_port():
                time.sleep(1)
                continue
            now = time.time()
            with self.lock:
                if now - self.last_input > INPUT_TIMEOUT:
                    self.speed = self.steer = 0
                    self.spin = False
                speed, steer, spin = self.speed, self.steer, self.spin
                cmds, self.pending = self.pending, []
            try:
                for c in cmds:
                    self._send(c)
                if spin:
                    # 탱크 믹싱: 속도는 앞뒤, 조향은 제자리 회전량으로 합침 (조향 서보는 중립)
                    w = steer * SPIN_MAX / 100
                    left = int(clamp(speed + w, -100, 100))
                    right = int(clamp(speed - w, -100, 100))
                    self._send(f"V {left} {right}")
                else:
                    self._send(f"D {speed} {steer}")
                if now - last_q >= STATUS_PERIOD:
                    self._send("?")
                    last_q = now
                self._read()
                # 블루투스 링크가 실제로 안 잡힌 채 포트만 열린 경우: 응답이 없으면 포트를 닫았다가 다시 연다
                if not self.sim and now - max(self.status_time, self.open_time) > NO_REPLY_REOPEN:
                    print(f"[serial] {NO_REPLY_REOPEN:.0f}초 동안 로봇 응답 없음 → 포트 재연결")
                    self._close()
                    time.sleep(1)
                    continue
            except (serial.SerialException, OSError) as e:
                print(f"[serial] 오류: {e}")
                self._close()
                continue
            next_t += period
            time.sleep(max(0.0, next_t - time.time()))
            if next_t < time.time() - 1:
                next_t = time.time()


# ---------------------------------------------------------------------------
async def index(request):
    return web.FileResponse(WEB_DIR / "index.html")


def handle_message(robot, m):
    t = m.get("t")
    if t == "drive":
        speed = clamp(int(m.get("speed", 0)), -100, 100)
        steer = clamp(int(m.get("steer", 0)), -100, 100)
        robot.set_drive(speed, steer, bool(m.get("spin", False)))
    elif t == "stop":
        robot.stop()
    elif t == "level":
        robot.queue(f"L {1 if m.get('on') else 0}")
    elif t == "height":
        robot.queue(f"H {clamp(int(m.get('v', 0)), -30, 30)}")


async def ws_handler(request):
    robot = request.app["robot"]
    ws = web.WebSocketResponse(heartbeat=10)
    await ws.prepare(request)
    request.app["clients"].add(ws)
    try:
        async for msg in ws:
            if msg.type == WSMsgType.TEXT:
                try:
                    handle_message(robot, json.loads(msg.data))
                except (ValueError, TypeError):
                    continue
    finally:
        request.app["clients"].discard(ws)
        robot.stop()   # 접속이 끊기면 정지
    return ws


async def broadcaster(app):
    robot = app["robot"]
    try:
        while True:
            await asyncio.sleep(0.25)
            snap = json.dumps(robot.snapshot())
            for ws in list(app["clients"]):
                try:
                    await ws.send_str(snap)
                except ConnectionError:
                    pass
    except asyncio.CancelledError:
        pass


async def on_startup(app):
    app["robot"].start()
    app["bc_task"] = asyncio.create_task(broadcaster(app))


async def on_cleanup(app):
    app["bc_task"].cancel()
    app["robot"].stop()
    time.sleep(0.2)
    app["robot"].running = False


def lan_ip():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("10.255.255.255", 1))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except OSError:
        return None


def main():
    ap = argparse.ArgumentParser(description="MOBED 웹 조종 서버")
    ap.add_argument("port", help="COM 포트 (예: COM9) 또는 SIM")
    ap.add_argument("--http-port", type=int, default=8000)
    ap.add_argument("--local", action="store_true", help="이 PC에서만 접속 허용")
    args = ap.parse_args()

    sim = args.port.upper() == "SIM"
    app = web.Application()
    app["robot"] = Robot(args.port, sim=sim)
    app["clients"] = set()
    app.router.add_get("/", index)
    app.router.add_get("/ws", ws_handler)
    app.on_startup.append(on_startup)
    app.on_cleanup.append(on_cleanup)

    host = "127.0.0.1" if args.local else "0.0.0.0"
    print(f"PC     : http://localhost:{args.http_port}")
    if not args.local:
        ip = lan_ip()
        if ip:
            print(f"폰     : http://{ip}:{args.http_port}   (같은 와이파이)")
        print("주의   : 같은 네트워크의 누구나 접속해 조종할 수 있음. 막으려면 --local")
    if sim:
        print("SIM 모드: 로봇 없이 명령만 출력")
    web.run_app(app, host=host, port=args.http_port, print=None)


if __name__ == "__main__":
    main()

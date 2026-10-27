"""
MOBED 키보드 조종 (Windows, 블루투스 COM 포트 또는 USB 시리얼)

사용법:  python teleop.py COM5
설치:    pip install pyserial

키:
  W / S   속도 +10 / -10 (%)
  A / D   조향 좌 / 우 (-10 / +10 %)
  C       조향 중립
  SPACE   정지 (속도 0, 조향 중립)
  Z / X   제자리 회전 좌 / 우
  Q / E   서스펜션 전체 높이 -5 / +5 (deg)
  L       자동 수평유지 켜기/끄기
  T       로봇 상태 조회 (연결/엔코더/PWM 확인)
  ESC     종료 (정지 후)
"""
import msvcrt
import sys
import time

import serial

if len(sys.argv) < 2:
    sys.exit("사용법: python teleop.py COM5")

ser = serial.Serial(sys.argv[1], 115200, timeout=0, write_timeout=0.2)
time.sleep(2)  # 포트 열릴 때 보드 리셋 대기

speed = 0
steer = 0
height = 0
level = False
spin = 0           # -1 / 0 / +1 (제자리 회전 중)
last_spin = 0.0
period = 0.05      # 명령 전송 주기 (20Hz)


def send(s):
    try:
        ser.write((s + "\n").encode())
    except serial.SerialTimeoutException:
        pass   # 블루투스 송신이 막히면 이번 명령은 건너뜀 (다음 주기에 다시 전송)


def show():
    mode = f"spin {spin:+d}" if spin else f"speed {speed:+4d}  steer {steer:+4d}"
    sys.stdout.write(f"\r{mode}  height {height:+3d}  level {'ON ' if level else 'OFF'}   ")
    sys.stdout.flush()


print(__doc__)
send("Q 0")
show()
next_t = time.time()
try:
    while True:
        while msvcrt.kbhit():
            ch = msvcrt.getwch()
            k = ch.lower()
            if ch == "\x1b":
                raise KeyboardInterrupt
            elif k == "w":
                speed = min(100, speed + 10); spin = 0
            elif k == "s":
                speed = max(-100, speed - 10); spin = 0
            elif k == "a":
                steer = max(-100, steer - 10); spin = 0
            elif k == "d":
                steer = min(100, steer + 10); spin = 0
            elif k == "c":
                steer = 0
            elif ch == " ":
                speed = steer = spin = 0
            elif k == "z":
                spin = -1; last_spin = time.time()
            elif k == "x":
                spin = 1; last_spin = time.time()
            elif k == "q":
                height = max(-30, height - 5); send(f"H {height}")
            elif k == "e":
                height = min(30, height + 5); send(f"H {height}")
            elif k == "t":
                send("?")
            elif k == "l":
                level = not level; send(f"L {int(level)}")
            show()

        # 제자리 회전은 키 반복 입력이 끊기면 멈춤 (누르고 있는 동안만)
        if spin and time.time() - last_spin > 0.3:
            spin = 0
            show()

        now = time.time()
        if now >= next_t:
            if spin:
                send(f"V {30 * spin} {-30 * spin}")
            else:
                send(f"D {speed} {steer}")
            next_t = now + period

        # 로봇이 돌려주는 메시지 표시
        data = ser.read(256)
        if data:
            sys.stdout.write("\n" + data.decode(errors="ignore").strip() + "\n")
            show()
        time.sleep(0.01)
except KeyboardInterrupt:
    pass
finally:
    send("X")
    time.sleep(0.1)
    ser.close()
    print("\n종료")

# MOBED 4WD 모바일 로봇

Arduino Mega 2560 + ESP32(블루투스)로 움직이는 4륜 구동 로봇의 펌웨어와 조종 프로그램입니다.

- **구동:** 좌/우 각각 모터 2개를 한 드라이버(BTS7960)로 구동하고, 엔코더로 **좌/우 속도 PID** 제어
- **조향:** 서보 2개(앞/뒤 조향, 기본은 뒷바퀴 역위상 4륜 조향)
- **서스펜션:** 서보 4개(코너별 높이), MPU9250으로 **자동 수평 유지**
- **조종:** PC나 폰에서 블루투스(ESP32)로 연결, 터미널 키보드(`teleop.py`) 또는 웹 조이스틱(`mobed_web.py`)

```
PC / 폰 ──블루투스(SPP)──▶ ESP32 ──UART(Serial2)──▶ Arduino Mega ──▶ 모터 드라이버, 서보, 엔코더, IMU
```

## 폴더 구성

| 파일 | 설명 |
|---|---|
| [Mobed.ino](Mobed.ino) | Arduino Mega 펌웨어 (속도 PID, 조향, 서스펜션, 수평 유지, 명령 처리) |
| [esp32_bridge/esp32_bridge.ino](esp32_bridge/esp32_bridge.ino) | ESP32 블루투스 ↔ 메가 시리얼 브리지 |
| [teleop.py](teleop.py) | 터미널 키보드 조종 |
| [mobed_web.py](mobed_web.py), [web/index.html](web/index.html) | 웹 조종 서버와 조이스틱 화면 (PC·폰) |
| [requirements.txt](requirements.txt) | 파이썬 패키지 목록 |

## 하드웨어

### 핀맵 (Arduino Mega)

| 기능 | 핀 |
|---|---|
| 엔코더 A상 FL / FR / RL / RR | D2 / D3 / D18 / D19 (외부 인터럽트) |
| 엔코더 B상 FL / FR / RL / RR | D22 / D23 / D24 / D25 |
| BTS7960 #1 (**좌측** 바퀴 2개) | RPWM D5, LPWM D6, L_EN D8, **R_EN D32** (D7이 고장 나서 D32 사용) |
| BTS7960 #2 (**우측** 바퀴 2개) | RPWM D9, LPWM D10, R_EN D11, L_EN D12 |
| ESP32 | D16 (TX2) → ESP32 RX2, D17 (RX2) ← ESP32 TX2 |
| MPU9250 | D20 (SDA), D21 (SCL), I2C 주소 0x68 (AD0 = GND) |
| 서보 1 / 2 | D26 / D27 : 앞 조향 / 뒤 조향 |
| 서보 3 / 4 / 5 / 6 | D28 / D29 / D30 / D31 : 서스펜션 FL / FR / RL / RR |

방향 기준은 **ESP32가 있는 쪽이 앞**입니다. 좌/우는 앞을 보고 섰을 때입니다.

### ESP32 배선

```
ESP32 GPIO16 (RX2) ◀── 분압 ── Arduino D16 (TX2)
ESP32 GPIO17 (TX2) ──────────▶ Arduino D17 (RX2)
GND 공통
```

Arduino의 TX 신호는 5V이므로 ESP32 RX 쪽은 **분압**해야 합니다 (예: D16 → 1kΩ → 접점(ESP32 GPIO16로) → 2kΩ → GND).
ESP32는 **원조 ESP32**(블루투스 클래식 SPP 지원)여야 합니다. S3나 C3는 이 코드로 안 됩니다.

### 전원 주의

- 서보와 ESP32 전원은 DC-DC 컨버터의 5V에서 받고, **Arduino 5V 핀에서 끌어쓰지 마세요.**
- Arduino를 DC 잭/VIN으로 외부 전원을 줄 때는 USB를 같이 꽂아도 됩니다. 5V 핀에 외부 5V를 넣었다면 USB를 같이 꽂지 마세요.
- 처음 구동 시험은 **바퀴를 바닥에서 띄운 상태**로 하세요.

## 설치

### 1. 클론

```
git clone <이 저장소 주소> Mobed
```

> Arduino IDE는 **스케치 폴더 이름이 .ino 파일 이름과 같아야** 합니다. 폴더 이름을 `Mobed`로 두세요.
> `esp32_bridge` 폴더는 같은 이유로 `esp32_bridge.ino`와 이름이 같습니다.

### 2. Arduino IDE 준비

1. [Arduino IDE](https://www.arduino.cc/en/software) 설치
2. 라이브러리 매니저에서 **Servo** 설치 (`Wire`는 기본 포함)
3. ESP32 보드 패키지 설치
   - 파일 → 환경설정 → "추가 보드 관리자 URL"에 아래 주소를 **추가** (다른 보드 URL이 있으면 쉼표로 구분해 같이 둠)
     `https://espressif.github.io/arduino-esp32/package_esp32_index.json`
   - 보드 매니저에서 `esp32` (Espressif Systems) 설치

### 3. 펌웨어 업로드

**Arduino Mega**
1. `Mobed.ino`를 연다. 보드는 **Arduino Mega or Mega 2560**, 포트는 메가의 COM 포트
2. 업로드. 부팅하면 "MOBED ready"가 시리얼(115200)에 찍힌다.
   IMU가 안 잡히면 "E imu not found"가 같이 나온다 (자동 수평만 안 되고 주행은 된다).
3. 모터 전원을 켠 채로 업로드하면 순간적으로 드라이버가 켜질 수 있으니 바퀴를 띄워 두세요.

**ESP32**
1. `esp32_bridge/esp32_bridge.ino`를 연다. 보드는 **ESP32 Dev Module**, 포트는 ESP32의 COM 포트
2. 업로드. 끝나고 시리얼 모니터(115200)에 "BT ready: MOBED"가 나오면 된다.
3. 업로드가 "Connecting..."에서 멈추면 ESP32의 **BOOT 버튼을 누른 채** 시도한다.
   USB는 데이터 케이블이어야 한다 (충전 전용 케이블이면 포트가 안 보인다).

### 4. 블루투스 페어링 (Windows)

1. 로봇과 ESP32 전원을 켠다.
2. 설정 → Bluetooth 및 장치 → 장치 추가 → Bluetooth → **MOBED** (비밀번호 없음)
3. 장치 관리자 → 포트(COM & LPT)에 "표준 Bluetooth에서 직렬 링크(COMx)"가 **두 개** 생긴다.
   이 중 **나가는(Outgoing)** 쪽 번호를 쓴다. 어느 쪽인지 모르겠으면 PowerShell에서 확인:
   ```
   Get-CimInstance Win32_PnPEntity | Where-Object { $_.DeviceID -like 'BTHENUM*' -and $_.Name -match 'COM\d+' } | Select-Object Name, DeviceID
   ```
   MOBED의 블루투스 주소가 들어 있는 줄이 나가는 포트다. (주소 부분이 `000000000000`이면 들어오는 포트)

### 5. 파이썬 패키지

Python 3.9 이상.
```
pip install -r requirements.txt
```

## 조종

> 블루투스 COM 포트는 한 번에 한 프로그램만 열 수 있습니다. `teleop.py`, 웹 서버, 시리얼 모니터를 동시에 같은 포트로 켜지 마세요.

### 터미널 키보드 (`teleop.py`)

```
python teleop.py COM9
```

| 키 | 동작 |
|---|---|
| W / S | 속도 +10% / -10% (누르면 유지) |
| A / D | 조향 좌 / 우 |
| C | 조향 중립 |
| Space | 정지 |
| Z / X | 제자리 회전 (누르는 동안) |
| Q / E | 서스펜션 높이 내리기 / 올리기 |
| L | 자동 수평 켜기/끄기 |
| T | 상태 조회 |
| Esc | 정지하고 종료 |

### 웹 조이스틱 (`mobed_web.py`)

```
python mobed_web.py COM9
```

- PC: `http://localhost:8000`
- 폰: 서버가 출력하는 `http://192.168.x.x:8000` (PC와 같은 와이파이, 방화벽에서 개인 네트워크 허용)
- 이 PC에서만 접속하게 하려면 `--local`을 붙인다. 로봇 없이 화면만 보려면 `python mobed_web.py SIM`.
- 화면: 가상 조이스틱(1개 / 2개 전환), 정지, **자동 수평 ON/OFF**, 제자리 회전(탱크 방식), 최대 속도와 서스펜션 높이 슬라이더, 상태 표시. 키보드와 게임패드도 지원한다.
- 같은 네트워크의 누구나 접속해 조종할 수 있으니 공용 네트워크에서는 `--local`을 쓰세요.

입력이 0.3초 끊기거나 접속이 끊기면 서버가 바로 정지시키고, 펌웨어에도 1초 명령 타임아웃이 따로 있습니다.

## 시리얼 명령

시리얼 모니터(USB, 115200, 줄 끝 "새 줄")나 블루투스 터미널에서 직접 보낼 수도 있습니다.

| 명령 | 동작 |
|---|---|
| `D <속도> <조향>` | 주행. 속도 -100~100 (% 최대 RPM), 조향 -100~100 (+ 우회전). **1초 안에 반복해서 보내야 계속 감** |
| `V <좌> <우>` | 좌/우 속도 직접 지정 (% 최대 RPM), 조향 서보는 중립 |
| `X` | 정지 |
| `H <높이>` / `H <FL> <FR> <RL> <RR>` | 서스펜션 높이 오프셋(도, ±30). + 이면 그 코너의 바퀴가 내려가 차체가 올라감 |
| `L <0\|1>` | 자동 수평 끄기/켜기 |
| `K <적분> [비례]` | 수평 유지 게인 (기본 8 / 3) |
| `P <kp> <ki> <kd>` | 바퀴 속도 PID 게인 |
| `S <1~6> <각도>` | 서보 하나를 원하는 각도(0~180)로 고정. `S 0`으로 해제 (캘리브레이션용) |
| `M <좌PWM> <우PWM>` | PID 없이 PWM 직접 출력 (-255~255, 3초 유지). 드라이버/방향 점검용 |
| `Q <0\|1>` | 상태 주기 출력 |
| `?` | 상태 출력 (`rpmL`, `rpmR`, `pwm`, `enc`, `pitch`, `roll`, `gap`, `cnt`, `bad` ...) |

## 내 로봇에 맞게 고칠 값

[Mobed.ino](Mobed.ino) 위쪽의 "사용자 설정" 구간입니다. 다른 로봇에 쓰거나 부품을 바꾸면 여기를 확인하세요.

| 상수 | 의미 |
|---|---|
| `ENC_PPR`, `GEAR_MOTOR`, `GEAR_EXTRA` | 엔코더 펄스 수(A상), 감속모터 감속비, 추가 기어비 → 바퀴 1회전당 카운트 |
| `MAX_RPM` | 속도 100%일 때 바퀴 RPM. 바닥에서 하중이 걸리면 PWM 여유가 남도록 정한다 |
| `WHEELBASE_MM`, `TRACK_MM` | 휠베이스, 트랙 (선회 시 좌/우 속도 차이 계산용. **기본값은 임시이니 실측값으로**) |
| `MOTOR_DIR` | 좌/우 모터 방향. 전진인데 바퀴가 반대로 돌면 부호를 바꾼다 |
| `ENC_DIR` | 엔코더 방향. 전진인데 `rpm`이 음수로 나오면 부호를 바꾼다 |
| `STEER_CENTER`, `STEER_DIR`, `MAX_STEER_DEG` | 조향 서보 중립, 방향, 최대 각도 |
| `SUS_CENTER`, `SUS_DIR`, `SUS_RANGE` | 서스펜션 서보 중립, 방향, 가동 범위 |
| `IMU_PITCH_SIGN`, `IMU_ROLL_SIGN` | IMU 장착 방향. 앞을 들었을 때 `pitch`가 +, 왼쪽을 들었을 때 `roll`이 +가 되어야 한다 |
| `levelP`, `levelK`, `LEVEL_LEAK_*` | 자동 수평 게인과 중립 복귀 속도 |
| `Kp`, `Ki`, `Kd` | 바퀴 속도 PID (런타임에 `P` 명령으로 조정 가능) |

### 처음 맞출 때 순서 (바퀴를 띄운 채로)

1. **좌/우 드라이버 확인:** `M 80 0` 하면 **좌측** 바퀴 앞뒤가, `M 0 80` 하면 **우측** 바퀴 앞뒤가 ESP32 쪽(앞)으로 돌아야 한다. 틀리면 `RPWM_PIN`/`LPWM_PIN`/`REN_PIN`/`LEN_PIN` 배열 순서나 `MOTOR_DIR`을 고친다.
2. **엔코더 방향:** `V 30 30`을 보내고 `?`로 `rpmL`, `rpmR`이 **둘 다 양수**인지 본다.
3. **PID:** `W`로 속도를 올렸을 때 `rpm`이 `tgt`를 안정적으로 따라가는지 본다. 흔들리면 `P` 명령으로 게인을 낮춘다.
4. **서보:** `S 1 90`처럼 하나씩 움직여 중립과 방향을 확인한다.
5. **IMU와 수평 유지:** 앞을 들면 `pitch`가 +, 왼쪽을 들면 `roll`이 +인지 확인한 뒤 `L 1`로 시험한다.

## 문제 해결

| 증상 | 확인 |
|---|---|
| 웹 화면이 "로봇 응답 없음" | 블루투스 링크가 안 잡힌 것. ESP32 전원, 재페어링, 나가는 COM 번호 확인. ESP32를 USB에 연결해 시리얼 모니터에서 `[BT connected]`가 뜨는지 본다 |
| `?`에 응답이 없음 | 줄 끝 설정이 "새 줄"인지, 보드레이트 115200인지, 메가가 부팅했는지("MOBED ready") |
| 달리다 뚝뚝 끊김 | `?`의 `gap`, `cnt`, `bad` 확인. `gap`이 크면 블루투스 지연, `bad`가 0이 아니면 글자가 깨짐 |
| 모터가 정지 명령에도 계속 돎 | `ENC_DIR` 부호가 반대일 가능성이 크다 (속도 제어가 거꾸로 동작) |
| IMU `imu=0` | SDA/SCL이 바뀌었는지, VCC/GND, AD0 확인 |
| ESP32 업로드가 "Connecting..."에서 실패 | BOOT 버튼을 누른 채로 시도, 데이터 케이블인지 확인 |

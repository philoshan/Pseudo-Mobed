/*
  MOBED-style 4WD mobile robot — Arduino Mega 2560

  구성
  - 구동 : DC모터 4개(엔코더 포함), BTS7960 2개 → 좌/우 각각 모터 2개 병렬
           좌/우 측 속도 PID (각 측 = 앞/뒤 엔코더 평균)
  - 조향 : 서보1(앞), 서보2(뒤) — 랙&피니언, 4WS(역위상) 또는 앞바퀴만
  - 서스 : 서보3~6 — 코너별 능동 서스펜션, MPU9250으로 차체 수평유지(옵션)
  - 통신 : ESP32(BT Classic) ↔ Serial2(D16 TX2 / D17 RX2), USB Serial도 같은 명령 사용

  좌표/방향 (도면 기준: 오른쪽 화살표 = 전진, 위 = 좌측, 아래 = 우측)
     서보3 = FL(좌전)   서보4 = FR(우전)
     서보5 = RL(좌후)   서보6 = RR(우후)
     서보1 = 앞 조향    서보2 = 뒤 조향

  명령 (줄 단위, '\n' 또는 '\r' 로 끝)
     D <speed> <steer>        주행: speed -100~100 (% MAX_RPM), steer -100~100 (% MAX_STEER_DEG, +는 우회전)
     V <left> <right>         좌/우 직접 (% MAX_RPM), 조향 중립 → 제자리 회전 가능
     X                        정지
     M <l_pwm> <r_pwm>        PID 없이 PWM 직접 출력 (-255~255, 3초 유지) — 배선/방향 점검용
     H <fl> <fr> <rl> <rr>    서스 높이 오프셋 (서보 각도 deg, +는 차체 올림, ±SUS_RANGE)
     H <all>                  4코너 동일 높이
     L <0|1>                  자동 수평유지 끄기/켜기
     P <kp> <ki> <kd>         속도 PID 게인 변경
     S <1~6> <angle>          서보 원시 각도 (캘리브레이션용, 0~180) / S 0 → 해제
     K <i> [p]                수평유지 게인: i=적분(기본 8), p=비례(기본 3)
     Q <0|1>                  텔레메트리 출력 끄기/켜기
     ?                        상태 출력
  D/V 명령은 CMD_TIMEOUT_MS 안에 반복해서 보내야 함 (끊기면 자동 정지)
*/

#include <Servo.h>
#include <Wire.h>
#include <math.h>

// =====================================================================
// 사용자 설정 (실측 후 수정)
// =====================================================================
const float ENC_PPR        = 13.0f;         // 엔코더 모터축 1회전당 A상 펄스 수 (데이터시트 26(13))
const float GEAR_MOTOR     = 100.0f;        // 감속모터 1/100
const float GEAR_EXTRA     = 4.0f;          // 추가 1:4 기어 (감속)
const float ENC_CPR        = ENC_PPR * GEAR_MOTOR * GEAR_EXTRA; // 바퀴 1회전당 A상 상승엣지 수
const float MAX_RPM        = 12.0f;         // speed 100% = 바퀴 RPM (모터 정격 52rpm / 4 = 13)
const float RAMP_RPM_PER_S = 24.0f;         // 가감속 제한
const float WHEELBASE_MM   = 300.0f;        // 앞/뒤 차축 간 거리
const float TRACK_MM       = 350.0f;        // 좌/우 바퀴 간 거리
const bool  FOUR_WS        = true;          // true: 뒤바퀴 역위상 조향, false: 앞바퀴만

const unsigned long CMD_TIMEOUT_MS = 1000;  // 주행명령 끊기면 정지 (블루투스 지연 고려)

// 방향 보정: 전진 명령 시 바퀴가 반대로 돌면 MOTOR_DIR, 전진인데 RPM이 음수면 ENC_DIR 부호 반전
const int MOTOR_DIR[2] = { -1, +1 };              // [LEFT, RIGHT]
const int ENC_DIR[4]   = { -1, -1, -1, +1 };      // [FL, FR, RL, RR]  전진(ESP32 쪽) 시 raw 카운트: FL- FR- RL- RR+ (실측 기반)

// 속도 PID (출력 단위: PWM 0~255)
float Kp = 4.0f, Ki = 8.0f, Kd = 0.0f;
const float KFF = 255.0f / MAX_RPM;               // 피드포워드 (대략 PWM/RPM)

// 조향 서보
const float MAX_STEER_DEG  = 25.0f;
const int   STEER_CENTER[2] = { 90, 90 };         // [앞(서보1), 뒤(서보2)]
const int   STEER_DIR[2]    = { +1, +1 };
const float STEER_GAIN      = 1.0f;               // 서보각 / 바퀴 조향각 (링크비)

// 서스펜션 서보
const float SUS_RANGE     = 30.0f;                // 높이 오프셋 한계 (deg)
const int   SUS_CENTER[4] = { 90, 90, 90, 90 };   // [FL, FR, RL, RR] 기본 높이
const int   SUS_DIR[4]    = { +1, -1, +1, -1 };   // 좌/우 미러 장착 가정

// 수평유지
float levelP               = 3.0f;                // 서보 deg per 기울기 deg (비례 게인, 기울기가 0이면 보정도 0)
float levelK               = 8.0f;                // 서보 deg/s per 기울기 deg (적분 게인)
const float LEVEL_LEAK_TAU = 8.0f;                // 기울어 있는 동안 적분항이 0으로 새는 시간상수 (s)
const float LEVEL_LEAK_FAST = 0.8f;               // 평평할 때(데드밴드 안) 적분항이 풀리는 시간상수 (s) — 작을수록 중립 복귀가 빠름
// 게인은 K <적분> <비례> 명령으로 변경
const float LEVEL_DEADBAND = 1.0f;                // deg
// IMU 장착 방향: 센서 X = 전진, Y = 좌측, Z = 위 가정. 반대면 부호 반전
const int IMU_PITCH_SIGN = +1;
const int IMU_ROLL_SIGN  = +1;

// =====================================================================
// 핀맵 (도면 기준)
// =====================================================================
enum { FL = 0, FR = 1, RL = 2, RR = 3 };
enum { LEFT = 0, RIGHT = 1 };

const uint8_t ENC_A_PIN[4] = { 2, 3, 18, 19 };    // 외부 인터럽트
const uint8_t ENC_B_PIN[4] = { 22, 23, 24, 25 };  // PORTA bit0~3 (PINA로 빠르게 읽음)

// BTS7960 실측 (M 명령 시험): #1(D5, D6, D8, R_EN=D32) = 좌측 바퀴 2개, #2(D9~D12) = 우측 바퀴 2개
// 배열 인덱스 [LEFT, RIGHT]
const uint8_t RPWM_PIN[2] = { 5, 9 };
const uint8_t LPWM_PIN[2] = { 6, 10 };
const uint8_t REN_PIN[2]  = { 32, 11 };   // #1의 R_EN은 D7 고장으로 D32 사용
const uint8_t LEN_PIN[2]  = { 8, 12 };

const uint8_t SERVO_PIN[6] = { 26, 27, 28, 29, 30, 31 }; // 서보1~6

#define BT_SERIAL Serial2   // D16 TX2 → ESP32 RX, D17 RX2 ← ESP32 TX
const long BT_BAUD  = 115200;
const long USB_BAUD = 115200;

const uint8_t MPU_ADDR = 0x68;

// =====================================================================
// 상태
// =====================================================================
volatile long encCount[4] = { 0, 0, 0, 0 };
long  prevCount[4] = { 0, 0, 0, 0 };
float wheelRpm[4]  = { 0, 0, 0, 0 };
float sideRpm[2]   = { 0, 0 };

float cmdRpm[2]  = { 0, 0 };   // 명령 목표
float rampRpm[2] = { 0, 0 };   // 램프 적용 목표
float integ[2]   = { 0, 0 };
float prevMeas[2] = { 0, 0 };
int   pwmOut[2]  = { 0, 0 };
bool  openLoop   = false;      // M 명령: PID 없이 PWM 직접 출력 (배선/방향 점검용)
int   olPwm[2]   = { 0, 0 };

float steerDeg = 0;                    // 바퀴 조향각 (+ 우회전)
float susHeight[4] = { 0, 0, 0, 0 };   // 사용자 높이 오프셋
float levelPitch = 0, levelRoll = 0;   // 수평유지 보정량 (비례항 + 적분항)
float levelPitchI = 0, levelRollI = 0; // 적분항
bool  levelOn = false;
int   servoOverride[6] = { -1, -1, -1, -1, -1, -1 };

bool  imuOk = false;
float gyroBias[3] = { 0, 0, 0 };
float pitchDeg = 0, rollDeg = 0;       // pitch + = 앞 들림, roll + = 좌측 들림

bool telemetryOn = false;
unsigned long lastCmdMs = 0;
unsigned long maxGapMs = 0;   // 주행명령(D/V) 사이 최대 간격 (? 출력 후 초기화)
unsigned int cmdCnt = 0, badCnt = 0;   // 받은 D/V 개수, 깨진 명령 개수 (? 출력 후 초기화)
char lastBad[24] = "";

Servo servo[6];

struct LineReader {
  char buf[64];
  uint8_t len = 0;
};
LineReader usbReader, btReader;

// =====================================================================
// 엔코더 ISR — A 상승엣지에서 B 상태로 방향 판단
// =====================================================================
void isrFL() { encCount[FL] += (PINA & _BV(0)) ? 1 : -1; }
void isrFR() { encCount[FR] += (PINA & _BV(1)) ? 1 : -1; }
void isrRL() { encCount[RL] += (PINA & _BV(2)) ? 1 : -1; }
void isrRR() { encCount[RR] += (PINA & _BV(3)) ? 1 : -1; }

// =====================================================================
// 모터
// =====================================================================
void setMotor(uint8_t side, int pwm) {
  pwm = constrain(pwm * MOTOR_DIR[side], -255, 255);
  if (pwm > 0) {
    analogWrite(LPWM_PIN[side], 0);
    analogWrite(RPWM_PIN[side], pwm);
  } else {
    analogWrite(RPWM_PIN[side], 0);
    analogWrite(LPWM_PIN[side], -pwm);
  }
}

void stopDrive() {
  cmdRpm[LEFT] = cmdRpm[RIGHT] = 0;
  openLoop = false;
  olPwm[LEFT] = olPwm[RIGHT] = 0;
}

// 속도(%)와 조향(%)으로 좌/우 목표 RPM 계산 (전자식 차동)
void setDrive(float speedPct, float steerPct) {
  speedPct = constrain(speedPct, -100, 100);
  steerPct = constrain(steerPct, -100, 100);
  steerDeg = steerPct / 100.0f * MAX_STEER_DEG;

  float v = speedPct / 100.0f * MAX_RPM;
  float t = tan(steerDeg * DEG_TO_RAD);
  // k = (TRACK/2) / R,  R = L/tanδ (앞바퀴만) 또는 (L/2)/tanδ (4WS)
  float k = (FOUR_WS ? TRACK_MM * t / WHEELBASE_MM : TRACK_MM * t / (2.0f * WHEELBASE_MM));
  float l = v * (1.0f + k);   // 우회전(+) → 좌측이 바깥
  float r = v * (1.0f - k);

  float m = max(fabs(l), fabs(r));
  if (m > MAX_RPM) { l *= MAX_RPM / m; r *= MAX_RPM / m; }
  cmdRpm[LEFT] = l;
  cmdRpm[RIGHT] = r;
}

void controlUpdate(float dt) {
  long c[4];
  noInterrupts();
  for (int i = 0; i < 4; i++) c[i] = encCount[i];
  interrupts();

  for (int i = 0; i < 4; i++) {
    long d = (c[i] - prevCount[i]) * ENC_DIR[i];
    prevCount[i] = c[i];
    float rpm = d / ENC_CPR / dt * 60.0f;
    wheelRpm[i] += 0.2f * (rpm - wheelRpm[i]);   // 저역통과 (저속에서 카운트 양자화 잡음 억제)
  }
  sideRpm[LEFT]  = 0.5f * (wheelRpm[FL] + wheelRpm[RL]);
  sideRpm[RIGHT] = 0.5f * (wheelRpm[FR] + wheelRpm[RR]);

  if (openLoop) {
    for (int s = 0; s < 2; s++) {
      pwmOut[s] = olPwm[s];
      setMotor(s, olPwm[s]);
      rampRpm[s] = 0;
      integ[s] = 0;
    }
    return;
  }

  float step = RAMP_RPM_PER_S * dt;
  for (int s = 0; s < 2; s++) {
    rampRpm[s] += constrain(cmdRpm[s] - rampRpm[s], -step, step);

    float meas = sideRpm[s];
    if (cmdRpm[s] == 0 && fabs(rampRpm[s]) < 0.3f) {   // 목표가 0이면 엔코더 값과 상관없이 모터 off (엔코더 방향 오류 시 폭주 방지)
      rampRpm[s] = 0;
      integ[s] = 0;
      prevMeas[s] = meas;
      pwmOut[s] = 0;
      setMotor(s, 0);
      continue;
    }

    float err = rampRpm[s] - meas;
    float dMeas = (meas - prevMeas[s]) / dt;
    prevMeas[s] = meas;

    float out = KFF * rampRpm[s] + Kp * err + integ[s] - Kd * dMeas;
    // 목표 방향의 반대(역방향) PWM은 쓰지 않음 → 브레이크성 진동 방지
    float lo = (cmdRpm[s] > 0) ? 0 : -255;
    float hi = (cmdRpm[s] < 0) ? 0 : 255;
    // 포화 시 같은 방향으로는 적분 안 함 (anti-windup)
    if (!((out > hi && err > 0) || (out < lo && err < 0))) {
      integ[s] = constrain(integ[s] + Ki * err * dt, -255, 255);
    }
    pwmOut[s] = (int)constrain(out, lo, hi);
    setMotor(s, pwmOut[s]);
  }
}

// =====================================================================
// 서보
// =====================================================================
void writeServo(int idx, float angle) {
  if (servoOverride[idx] >= 0) angle = servoOverride[idx];
  servo[idx].write((int)constrain(angle + 0.5f, 0, 180));
}

void servoUpdate() {
  float a = steerDeg * STEER_GAIN;
  writeServo(0, STEER_CENTER[0] + STEER_DIR[0] * a);                    // 앞 조향
  writeServo(1, STEER_CENTER[1] + STEER_DIR[1] * (FOUR_WS ? -a : 0));   // 뒤 조향

  for (int i = 0; i < 4; i++) {
    bool front = (i == FL || i == FR);
    bool left  = (i == FL || i == RL);
    float h = susHeight[i]
              + (front ? -levelPitch : levelPitch)
              + (left ? -levelRoll : levelRoll);
    h = constrain(h, -SUS_RANGE, SUS_RANGE);
    writeServo(2 + i, SUS_CENTER[i] + SUS_DIR[i] * h);
  }
}

// =====================================================================
// IMU (MPU9250 가속도/자이로만 사용, 상보필터)
// =====================================================================
void mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

bool mpuRead(int16_t *raw) {   // ax ay az temp gx gy gz
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU_ADDR, (uint8_t)14) != 14) return false;
  for (int i = 0; i < 7; i++) {
    uint8_t hi = Wire.read();
    uint8_t lo = Wire.read();
    raw[i] = (int16_t)((hi << 8) | lo);
  }
  return true;
}

void imuInit() {
  Wire.begin();
  Wire.setClock(400000);
#if defined(WIRE_HAS_TIMEOUT)
  Wire.setWireTimeout(3000, true);
#endif
  mpuWrite(0x6B, 0x00);   // 슬립 해제
  delay(100);
  mpuWrite(0x1A, 0x03);   // DLPF 44Hz
  mpuWrite(0x1B, 0x00);   // 자이로 ±250dps
  mpuWrite(0x1C, 0x00);   // 가속도 ±2g

  int16_t raw[7];
  long sum[3] = { 0, 0, 0 };
  const int N = 300;
  for (int i = 0; i < N; i++) {
    if (!mpuRead(raw)) { imuOk = false; return; }
    for (int k = 0; k < 3; k++) sum[k] += raw[4 + k];
    delay(3);
  }
  for (int k = 0; k < 3; k++) gyroBias[k] = sum[k] / (float)N;

  mpuRead(raw);
  pitchDeg = IMU_PITCH_SIGN * atan2(raw[0], sqrt((float)raw[1] * raw[1] + (float)raw[2] * raw[2])) * RAD_TO_DEG;
  rollDeg  = IMU_ROLL_SIGN * atan2(raw[1], raw[2]) * RAD_TO_DEG;
  imuOk = true;
}

void imuUpdate(float dt) {
  int16_t raw[7];
  if (!mpuRead(raw)) return;
  float ax = raw[0], ay = raw[1], az = raw[2];
  float gx = (raw[4] - gyroBias[0]) / 131.0f;
  float gy = (raw[5] - gyroBias[1]) / 131.0f;

  float accPitch = IMU_PITCH_SIGN * atan2(ax, sqrt(ay * ay + az * az)) * RAD_TO_DEG;
  float accRoll  = IMU_ROLL_SIGN * atan2(ay, az) * RAD_TO_DEG;
  // X=전진, Y=좌측 기준: 앞 들림 속도 = -gy, 좌측 들림 속도 = +gx
  pitchDeg = 0.98f * (pitchDeg + IMU_PITCH_SIGN * -gy * dt) + 0.02f * accPitch;
  rollDeg  = 0.98f * (rollDeg + IMU_ROLL_SIGN * gx * dt) + 0.02f * accRoll;

  if (levelOn) {
    float ep = (fabs(pitchDeg) > LEVEL_DEADBAND) ? pitchDeg : 0;
    float er = (fabs(rollDeg) > LEVEL_DEADBAND)  ? rollDeg : 0;
    // 적분항: 기울기 동안 쌓이고, 시간상수 LEVEL_LEAK_TAU 로 서서히 0으로 복귀
    float tauP = (ep == 0) ? LEVEL_LEAK_FAST : LEVEL_LEAK_TAU;
    float tauR = (er == 0) ? LEVEL_LEAK_FAST : LEVEL_LEAK_TAU;
    levelPitchI += (levelK * ep - levelPitchI / tauP) * dt;
    levelRollI  += (levelK * er - levelRollI / tauR) * dt;
    levelPitchI = constrain(levelPitchI, -SUS_RANGE, SUS_RANGE);
    levelRollI  = constrain(levelRollI, -SUS_RANGE, SUS_RANGE);
    levelPitch  = constrain(levelP * ep + levelPitchI, -SUS_RANGE, SUS_RANGE);
    levelRoll   = constrain(levelP * er + levelRollI, -SUS_RANGE, SUS_RANGE);
  }
}

// =====================================================================
// 명령 처리
// =====================================================================
void printStatus(Stream &out) {
  out.print(F("S rpmL=")); out.print(sideRpm[LEFT], 1);
  out.print(F(" rpmR="));  out.print(sideRpm[RIGHT], 1);
  out.print(F(" tgtL="));  out.print(rampRpm[LEFT], 1);
  out.print(F(" tgtR="));  out.print(rampRpm[RIGHT], 1);
  out.print(F(" pwm="));   out.print(pwmOut[LEFT]); out.print('/'); out.print(pwmOut[RIGHT]);
  out.print(F(" enc="));
  for (int i = 0; i < 4; i++) { out.print(encCount[i]); out.print(i < 3 ? ',' : ' '); }
  out.print(F("steer=")); out.print(steerDeg, 1);
  out.print(F(" pitch=")); out.print(pitchDeg, 1);
  out.print(F(" roll="));  out.print(rollDeg, 1);
  out.print(F(" level=")); out.print(levelOn);
  out.print(F(" imu="));   out.print(imuOk);
  out.print(F(" gap="));   out.print(maxGapMs);
  out.print(F(" cnt="));   out.print(cmdCnt);
  out.print(F(" bad="));   out.print(badCnt);
  if (badCnt) { out.print(F(" last=[")); out.print(lastBad); out.print(']'); }
  out.println();
  maxGapMs = 0; cmdCnt = 0; badCnt = 0;
}

void noteCmdGap() {   // 명령 간 간격 기록 (오래 쉬었다 보낸 경우는 제외)
  unsigned long g = millis() - lastCmdMs;
  if (g < 3000) maxGapMs = max(maxGapMs, g);
  cmdCnt++;
}

void markBad(const char *raw) {
  badCnt++;
  strncpy(lastBad, raw, sizeof(lastBad) - 1);
  lastBad[sizeof(lastBad) - 1] = ' ';
}

void handleCommand(char *line, Stream &out) {
  float a[4];
  int n = 0;
  char raw[24];
  strncpy(raw, line, sizeof(raw) - 1);
  raw[sizeof(raw) - 1] = ' ';
  char *tok = strtok(line, " ,\t");
  if (!tok) return;
  char cmd = toupper(tok[0]);
  while (n < 4 && (tok = strtok(NULL, " ,\t"))) a[n++] = atof(tok);

  switch (cmd) {
    case 'M':   // M <left_pwm> <right_pwm> : 좌/우 채널 PWM 직접 출력 (-255~255), 3초 유지
      if (n >= 2) {
        olPwm[LEFT]  = constrain((int)a[0], -255, 255);
        olPwm[RIGHT] = constrain((int)a[1], -255, 255);
        openLoop = true;
        lastCmdMs = millis();
      }
      break;
    case 'D':
      openLoop = false;
      if (n >= 2) { noteCmdGap(); setDrive(a[0], a[1]); lastCmdMs = millis(); }
      else markBad(raw);
      break;
    case 'V':
      openLoop = false;
      if (n >= 2) {
        noteCmdGap();
        steerDeg = 0;
        cmdRpm[LEFT]  = constrain(a[0], -100, 100) / 100.0f * MAX_RPM;
        cmdRpm[RIGHT] = constrain(a[1], -100, 100) / 100.0f * MAX_RPM;
        lastCmdMs = millis();
      } else markBad(raw);
      break;
    case 'X':
      stopDrive();
      break;
    case 'H':
      if (n == 1) for (int i = 0; i < 4; i++) susHeight[i] = constrain(a[0], -SUS_RANGE, SUS_RANGE);
      else if (n >= 4) for (int i = 0; i < 4; i++) susHeight[i] = constrain(a[i], -SUS_RANGE, SUS_RANGE);
      break;
    case 'L':
      if (n >= 1) {
        levelOn = (a[0] != 0);
        if (!levelOn) levelPitch = levelRoll = levelPitchI = levelRollI = 0;
        if (levelOn && !imuOk) out.println(F("E imu not found"));
      }
      break;
    case 'P':
      if (n >= 3) { Kp = a[0]; Ki = a[1]; Kd = a[2]; integ[0] = integ[1] = 0; }
      out.print(F("PID ")); out.print(Kp, 3); out.print(' '); out.print(Ki, 3); out.print(' '); out.println(Kd, 3);
      break;
    case 'S':
      if (n >= 1 && (int)a[0] == 0) for (int i = 0; i < 6; i++) servoOverride[i] = -1;
      else if (n >= 2 && a[0] >= 1 && a[0] <= 6) servoOverride[(int)a[0] - 1] = constrain((int)a[1], 0, 180);
      break;
    case 'K':   // K <적분게인> [비례게인] : 수평유지 게인
      if (n >= 1) levelK = constrain(a[0], 0, 50);
      if (n >= 2) levelP = constrain(a[1], 0, 10);
      out.print(F("levelK ")); out.print(levelK, 2);
      out.print(F(" levelP ")); out.println(levelP, 2);
      break;
    case 'Q':
      if (n >= 1) telemetryOn = (a[0] != 0);
      break;
    case '?':
      printStatus(out);
      break;
    default:
      markBad(raw);
      out.println(F("E unknown"));
      break;
  }
}

void pollSerial(Stream &port, LineReader &r) {
  while (port.available()) {
    char c = port.read();
    if (c == '\n' || c == '\r') {
      if (r.len > 0) {
        r.buf[r.len] = '\0';
        handleCommand(r.buf, port);
        r.len = 0;
      }
    } else if (r.len < sizeof(r.buf) - 1) {
      r.buf[r.len++] = c;
    } else {
      r.len = 0;   // 너무 긴 줄은 버림
    }
  }
}

// =====================================================================
void setup() {
  Serial.begin(USB_BAUD);
  BT_SERIAL.begin(BT_BAUD);

  for (int s = 0; s < 2; s++) {
    pinMode(RPWM_PIN[s], OUTPUT);
    pinMode(LPWM_PIN[s], OUTPUT);
    pinMode(REN_PIN[s], OUTPUT);
    pinMode(LEN_PIN[s], OUTPUT);
    setMotor(s, 0);
    digitalWrite(REN_PIN[s], HIGH);
    digitalWrite(LEN_PIN[s], HIGH);
  }

  for (int i = 0; i < 4; i++) {
    pinMode(ENC_A_PIN[i], INPUT_PULLUP);
    pinMode(ENC_B_PIN[i], INPUT_PULLUP);
  }
  attachInterrupt(digitalPinToInterrupt(ENC_A_PIN[FL]), isrFL, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_A_PIN[FR]), isrFR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_A_PIN[RL]), isrRL, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_A_PIN[RR]), isrRR, RISING);

  for (int i = 0; i < 6; i++) servo[i].attach(SERVO_PIN[i]);
  servoUpdate();

  imuInit();   // 부팅 시 로봇을 가만히 둘 것 (자이로 바이어스 측정)

  Serial.println(F("MOBED ready"));
  BT_SERIAL.println(F("MOBED ready"));
  if (!imuOk) Serial.println(F("E imu not found"));
}

void loop() {
  static unsigned long tCtrl = micros(), tImu = micros();
  static unsigned long tServo = millis(), tTele = millis();

  pollSerial(Serial, usbReader);
  pollSerial(BT_SERIAL, btReader);

  // 시각은 명령 처리(lastCmdMs 갱신) 뒤에 읽어야 함. 먼저 읽으면 nowMs - lastCmdMs 가 언더플로우되어 오탐 정지가 발생
  unsigned long nowUs = micros(), nowMs = millis();

  if ((cmdRpm[LEFT] != 0 || cmdRpm[RIGHT] != 0) && nowMs - lastCmdMs > CMD_TIMEOUT_MS) {
    stopDrive();
  }
  if (openLoop && nowMs - lastCmdMs > 3000) stopDrive();

  if (nowUs - tCtrl >= 20000) {          // 50 Hz 속도제어
    controlUpdate((nowUs - tCtrl) * 1e-6f);
    tCtrl = nowUs;
  }
  if (imuOk && nowUs - tImu >= 10000) {  // 100 Hz IMU
    imuUpdate((nowUs - tImu) * 1e-6f);
    tImu = nowUs;
  }
  if (nowMs - tServo >= 20) {            // 50 Hz 서보
    servoUpdate();
    tServo = nowMs;
  }
  if (telemetryOn && nowMs - tTele >= 200) {
    printStatus(BT_SERIAL);
    printStatus(Serial);
    tTele = nowMs;
  }
}

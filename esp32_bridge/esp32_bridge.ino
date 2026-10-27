/*
  ESP32 Bluetooth Classic(SPP) ↔ Arduino Mega Serial2 브리지
  - 원조 ESP32(WROOM/WROVER)만 BT Classic 지원 (S3/C3는 불가)
  - 배선 (UART2 사용): ESP32 RX2(GPIO16) ← Mega D16 (TX2)  ※ 5V→3.3V 분압 필요 (예: 1kΩ/2kΩ)
                       ESP32 TX2(GPIO17) → Mega D17 (RX2)
                       GND 공통
  - UART0(USB)은 업로드/시리얼 모니터 디버깅 전용으로 남겨 둠
*/
#include <BluetoothSerial.h>

BluetoothSerial SerialBT;

const int  MEGA_RX = 16;   // ESP32 RX2
const int  MEGA_TX = 17;   // ESP32 TX2
const long MEGA_BAUD = 115200;

void setup() {
  Serial.begin(115200);
  Serial2.begin(MEGA_BAUD, SERIAL_8N1, MEGA_RX, MEGA_TX);
  SerialBT.begin("MOBED");
  SerialBT.register_callback([](esp_spp_cb_event_t e, esp_spp_cb_param_t *) {
    if (e == ESP_SPP_SRV_OPEN_EVT) Serial.println("[BT connected]");
    if (e == ESP_SPP_CLOSE_EVT) Serial.println("[BT disconnected]");
  });
  Serial.println("BT ready: MOBED");
}

void loop() {
  while (SerialBT.available()) {
    int c = SerialBT.read();
    Serial2.write(c);
    Serial.write(c);   // 디버그: PC에서 블루투스로 받은 글자를 USB 시리얼에도 표시
  }
  while (Serial2.available()) {
    int c = Serial2.read();
    SerialBT.write(c);
    Serial.write(c);
  }
}

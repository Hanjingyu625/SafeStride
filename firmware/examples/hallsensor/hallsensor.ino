const byte SENSOR_PIN = 2; // 홀 센서 OUT (디지털 2번 핀)
const byte LED_PIN = 5;    // LED 연결 핀

// 측정 시간 변수
volatile unsigned long last_time = 0;
volatile unsigned long time_diff = 0;
volatile bool new_pulse = false;

// 바퀴 및 환경 설정 (필요에 맞게 수정)
const float WHEEL_RADIUS_M = 0.03; // 바퀴 반지름 (예: 3cm = 0.03m)
const float WHEEL_CIRCUMFERENCE = 2 * 3.141592 * WHEEL_RADIUS_M; // 바퀴 둘레 (m)
const int MAGNET_COUNT = 1; // 바퀴에 붙인 자석 개수

// 인터럽트 함수 (자석 감지 시 자동 실행)
void sensorISR() {
  unsigned long current_time = micros();
  unsigned long duration = current_time - last_time;

  // [핵심 디바운싱]: 50ms(50,000us) 이내의 너무 빠른 신호 변화는 노이즈로 간주해 무시
  if (duration > 50000) { 
    time_diff = duration;
    last_time = current_time;
    new_pulse = true;
  }
}

void setup() {
  Serial.begin(9600);
  pinMode(LED_PIN, OUTPUT);
  pinMode(SENSOR_PIN, INPUT_PULLUP);

  // 센서 신호가 LOW로 떨어질 때(자석 감지) 인터럽트 발생
  attachInterrupt(digitalPinToInterrupt(SENSOR_PIN), sensorISR, FALLING);
  
  Serial.println("--- 속도 및 RPM 측정 시작 ---");
}

void loop() {
  // 정상적인 자석 통과 신호가 들어왔을 때만 실행
  if (new_pulse) {
    noInterrupts(); // 계산 중 인터럽트 중단
    unsigned long duration = time_diff;
    new_pulse = false;
    interrupts();

    // 1회전에 걸린 시간 계산 (초 단위)
    float rev_time_sec = ((float)duration / 1000000.0) * MAGNET_COUNT;

    // 1. RPM 계산
    float rpm = (1.0 / rev_time_sec) * 60.0;

    // 2. 이동 속도 계산 (km/h)
    float speed_mps = WHEEL_CIRCUMFERENCE / rev_time_sec;
    float speed_kmh = speed_mps * 3.6;

    // 결과 출력
    Serial.print("RPM: ");
    Serial.print(rpm, 1);
    Serial.print(" | 속도: ");
    Serial.print(speed_kmh, 2);
    Serial.println(" km/h");

    // 자석이 정상 감지되었을 때만 LED가 1번 깜빡임
    digitalWrite(LED_PIN, HIGH);
    delay(30);
    digitalWrite(LED_PIN, LOW);
  }

  // 2초 이상 자석이 감지되지 않으면 정지 상태로 판단
  if (micros() - last_time > 2000000 && last_time != 0) {
    Serial.println("RPM: 0.0 | 속도: 0.00 km/h (정지)");
    last_time = micros();
  }

  delay(10);
}
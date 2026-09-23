# Terrain Uno firmware

ezHMI는 AltSoftSerial 1.4.0으로 D9(TX)→DIN, D8(RX)←DOUT에 연결한다.
LCD의 VisualTFT 최초 구성과 배포 절차는 [디스플레이 안내](../../docs/DISPLAY_KO.md)를 따른다.
설치: `arduino-cli lib install AltSoftSerial@1.4.0`.

TOF-10120과 GY-521 MPU6050을 읽어 protocol v6 텔레메트리로 보낸다. BE-220
GPS는 Raspberry Pi의 `gps_node`가 별도 serial 장치로 직접 수신한다.

- A4/A5: TOF `0x52`, MPU6050 `0x68` 또는 `0x69`
- 주기: TOF/MPU 50 ms

## CdS 조명 제어 (출력 활성화 전)

A0에서 CdS 분압 전압을 50 ms마다 읽고, Terrain 내부에서 조명 ON/OFF를
판단한다. Pi 연결 여부나 주행 상태에 종속되지 않으며 `delay()`를 사용하지 않는다.
릴레이 핀과 ON 레벨은 미정이므로 현재 `LIGHT_OUTPUT_ENABLED=false`,
`LIGHT_RELAY_PIN=-1`, `LIGHT_RELAY_ON_LEVEL=-1`이다. **현재 펌웨어는 릴레이
핀을 설정하거나 구동하지 않는다.** A0 샘플링과 논리 판단만 수행한다.

- 설정 위치: `config.h`의 `LIGHT_*`.
- 밝기 ADC가 350 이하로 1초 유지되면 ON, 550 이상으로 1초 유지되면 OFF.
  중간 구간에서는 이전 상태를 유지하며 조건이 깨지면 대기 시간을 초기화한다.
  이 수치들은 미튜닝 임시값이며 lux가 아니다. 부팅 시 논리 상태는 OFF이다.
- 예정 배선: `5V -- CdS -- A0 -- 고정저항 -- GND` (밝을수록 ADC 증가).
  고정저항 값은 추후 밝기 관찰로 결정한다. CdS 단품은 분압 회로가 필요하다.
  반대 분압 배선이면 `LIGHT_BRIGHT_IS_HIGH=false`로 바꾼다.
- 릴레이 활성화: 실제 핀과 HIGH/LOW 트리거를 확인하고 위 세 설정을 변경한다.
  D0/D1(USB), D8/D9(LCD), A4/A5(I2C)는 사용하지 않는다. 출력 활성화 시
  OFF 레벨을 먼저 기록한 다음 OUTPUT으로 설정한다. 리셋 중 핀은 입력 상태이므로
  해당 릴레이 모듈의 기본 OFF 동작은 실제 하드웨어에서 별도로 확인한다.
- 전력 경로: 배터리 + → 릴레이 COM/NO → 조명 +, 조명 - → 배터리 -.
  제어 경로: Terrain 출력 → 릴레이 IN, 모듈의 전원/접지는 실제 모듈 사양에 맞춘다.
  배터리/조명의 전압·전류와 릴레이 DC 정격은 아직 확인하지 않았다.
- 추후 관찰용 접근자: `g_light.rawAdc()`, `hasSample()`, `requestedOn()`,
  `outputReady()`. `requestedOn()`은 논리 요구 상태이며 실제 점등 피드백이 아니다.
  Pi 프로토콜, LCD 표시, USB 직렬 텍스트 출력은 추가하지 않았다.
  센서 단선과 정상적인 극단 밝기는 ADC만으로 구분하지 않는다.

제품 참고: [5V 1채널 릴레이](https://www.coupang.com/vp/products/70339702),
[5528 CdS](https://www.coupang.com/vp/products/39161074).
상품명만 확인했으며 지정 판매 옵션의 트리거 극성은 미확인이다.
호스트 테스트는 전환 지연·히스테리시스·극성·출력 비활성화·시간 오버플로를
검사한다. 밝기 튜닝, 실제 배선과 점등 시험은 후속 작업이다.

TOF는 높이 442 mm / 아래로 45°의 고정 기준 약 625.1 mm를 사용한다.
10샘플 필터 준비 후 EMA alpha 0.3, 광선 거리 변화 약 353.6 mm(수직 250 mm),
같은 방향 4회로 raised/drop을 확정한다. 확정 결과는 최소 1초 유지한다.
측정 불가는 진단만 전송한다. 시스템의 3초 PWM 감속과 자동 재출발은
[ToF 정지 안내](../../docs/TOF_STOP_KO.md)를 참고한다.

MPU6050은 ±2 g, ±250 deg/s, 20 Hz 출력으로 설정한다. roll/pitch는 가속도
중력 방향으로 계산해 EMA로 평활하며 yaw는 제공하지 않는다. 3회 연속 I2C 읽기
실패 시 센서를 다시 검색하고, 재연결 뒤 첫 샘플로 자세 필터를 초기화한다.

```bash
arduino-cli compile --fqbn arduino:avr:uno firmware/terrain_mcu
arduino-cli upload --fqbn arduino:avr:uno -p /dev/safestride-terrain \
  firmware/terrain_mcu
```

# Terrain Uno firmware

ezHMI는 AltSoftSerial 1.4.0으로 D9(TX)→DIN, D8(RX)←DOUT에 연결한다.
LCD의 VisualTFT 최초 구성과 배포 절차는 [디스플레이 안내](../../docs/DISPLAY_KO.md)를 따른다.
설치: `arduino-cli lib install AltSoftSerial@1.4.0`.

TOF-10120과 GY-521 MPU6050을 읽어 protocol v6 텔레메트리로 보낸다. BE-220
GPS는 Raspberry Pi의 `gps_node`가 별도 serial 장치로 직접 수신한다.

- A4/A5: TOF `0x52`, MPU6050 `0x68` 또는 `0x69`
- 주기: TOF/MPU 50 ms

## CdS 조명 제어 (D4 출력 활성화)

A0에서 CdS 분압 전압을 50 ms마다 읽고, Terrain 내부에서 조명 ON/OFF를
판단한다. Pi 연결 여부나 주행 상태에 종속되지 않으며 `delay()`를 사용하지 않는다.
`LIGHT_OUTPUT_ENABLED=true`, `LIGHT_RELAY_PIN=4`,
`LIGHT_RELAY_ON_LEVEL=HIGH`로 설정되어 있다. D4 HIGH에서 ON, LOW에서 OFF이다.
극성은 아래 판매자 예제를 기준으로 하며 실제 모듈의 점등 시험은 아직 수행하지 않았다.
설정은 Terrain Uno 펌웨어를 다시 빌드·업로드해야 적용된다.

- 설정 위치: `config.h`의 `LIGHT_*`.
- 밝기 ADC가 350 이하로 1초 유지되면 ON, 550 이상으로 1초 유지되면 OFF.
  중간 구간에서는 이전 상태를 유지하며 조건이 깨지면 대기 시간을 초기화한다.
  이 수치들은 미튜닝 임시값이며 lux가 아니다. 부팅 시 논리 상태는 OFF이다.
- 확인된 배선: `5V -- CdS -- A0 -- 10kΩ -- GND` (밝을수록 ADC 증가).
  반대 분압 배선이면 `LIGHT_BRIGHT_IS_HIGH=false`로 바꾼다.
- 릴레이 출력: D4 → IN(신호), VCC → 5V, GND → 공통 GND.
  D0/D1(USB), D8/D9(LCD), A4/A5(I2C)는 사용하지 않는다. 출력 활성화 시
  OFF 레벨을 먼저 기록한 다음 OUTPUT으로 설정한다. 리셋 중 핀은 입력 상태이므로
  해당 릴레이 모듈의 기본 OFF 동작은 실제 하드웨어에서 별도로 확인한다.
- 전력 경로: 배터리 + → 릴레이 COM/NO → 조명 +, 조명 - → 배터리 -.
  COM–NO 연결은 사용자 확인 사항이다. 배터리/조명의 실제 전압·전류는 미확인이다.
- 추후 관찰용 접근자: `g_light.rawAdc()`, `hasSample()`, `requestedOn()`,
  `outputReady()`. `requestedOn()`은 논리 요구 상태이며 실제 점등 피드백이 아니다.
  Pi 프로토콜, LCD 표시, USB 직렬 텍스트 출력은 추가하지 않았다.
  센서 단선과 정상적인 극단 밝기는 ADC만으로 구분하지 않는다.

사용자 지정 제품: [쿠팡 1채널 5V 릴레이](https://www.coupang.com/vp/products/7495734120?itemId=19610387283&vendorItemId=86717369619).
본체 표기는 SRD-05VDC-SL-C이며 이것만으로 모듈의 트리거 극성을 결정할 수 없다.
쿠팡 판매자(로보다인시스템)와 상품명이 일치하는
[에듀이노 D-41 안내](https://m.eduino.kr/product/detail.html?product_no=56)의 예제는
HIGH=ON, LOW=OFF이다. 이를 근거로 HIGH 설정을 유지한다. 쿠팡 상세 이미지에는
접근하지 못했으므로 동일 모듈이라는 판단은 판매자·상품명 대조에 기반한 추정이다.
이전 SZH-EK093용 H/L 점퍼 설정 안내는 적용하지 않는다.
실물에서 반대로 동작하면 `LIGHT_RELAY_ON_LEVEL=LOW`로 변경하고 재빌드·업로드한다.
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

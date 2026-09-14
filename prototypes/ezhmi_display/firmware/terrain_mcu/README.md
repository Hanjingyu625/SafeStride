# Terrain Uno firmware

TOF-10120과 GY-521 MPU6050을 읽어 protocol v6 텔레메트리로 보낸다. BE-220
GPS는 Raspberry Pi의 `gps_node`가 별도 serial 장치로 직접 수신한다.

- A4/A5: TOF `0x52`, MPU6050 `0x68` 또는 `0x69`
- 주기: TOF/MPU 50 ms

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

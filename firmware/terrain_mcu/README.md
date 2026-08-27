# Terrain Uno firmware

TOF-10120, GY-521 MPU6050과 BE-220 GPS를 읽어 protocol v4 텔레메트리로 보낸다.

- A4/A5: TOF `0x52`, MPU6050 `0x68` 또는 `0x69`
- D8/D9: BE-220 AltSoftSerial RX/TX, 9600 baud
- 주기: TOF/MPU 50 ms, GPS parser 상시 poll

TOF는 10샘플 기준면을 만든 뒤 EMA alpha 0.3, adaptive reference alpha 0.02,
error 60 mm, change 10 mm, 같은 방향 4회로 raised/drop을 확정한다. 후보·확정
중에는 기준값 갱신을 멈추고 확정 결과는 최소 1초 유지한다.

MPU6050은 ±2 g, ±250 deg/s로 설정한다. roll/pitch는 가속도 중력 방향으로
계산하며 yaw는 제공하지 않는다.

```bash
arduino-cli compile --fqbn arduino:avr:uno firmware/terrain_mcu
arduino-cli upload --fqbn arduino:avr:uno -p /dev/safestride-terrain \
  firmware/terrain_mcu
```

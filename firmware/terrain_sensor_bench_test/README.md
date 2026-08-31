# Terrain sensor bench

Terrain Uno의 TOF-10120과 GY-521 MPU6050만 독립 확인하는 스케치다.

| Uno | 연결 |
|---|---|
| A4/SDA | TOF-10120 SDA, GY-521 SDA |
| A5/SCL | TOF-10120 SCL, GY-521 SCL |
| GND | 두 센서 공통 GND |

TOF 주소는 `0x52`, GY-521은 AD0 상태에 따라 `0x68` 또는 `0x69`다. 업로드 후
115200 baud 시리얼 모니터에서 거리와 3축 가속도·자이로 원시값이 움직임에 따라
변하는지 확인한다.

```bash
arduino-cli compile --fqbn arduino:avr:uno firmware/terrain_sensor_bench_test
```

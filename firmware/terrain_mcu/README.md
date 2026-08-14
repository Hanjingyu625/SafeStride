# Terrain MCU 펌웨어

Arduino Uno가 TOF-10120을 I2C 주소 `0x52`에서 50 ms마다 읽고 프로토콜 v2
텔레메트리로 Raspberry Pi에 전송한다. 상태 LED 출력은 사용하지 않으며
결과는 `/terrain/tof`, `/terrain/status`, `/diagnostics`에서 확인한다.

## 핀맵

| 기능 | Uno 핀 |
|---|---:|
| I2C SDA | A4 |
| I2C SCL | A5 |
| 센서 공통 GND | GND |

TOF 유효 범위는 100~2000 mm이다. 필터 거리와 느린 기준 거리의 차이가
60 mm를 넘으면 후보가 되고, 10 mm 이상의 상승이 4회 연속되면 단차로
판정한다. LED 대신 모든 상태를 직렬 텔레메트리와 ROS 토픽으로만 보낸다.

```bash
arduino-cli compile --fqbn arduino:avr:uno firmware/terrain_mcu
arduino-cli upload --fqbn arduino:avr:uno -p /dev/safestride-terrain \
  firmware/terrain_mcu
```

현재 운영 펌웨어의 ROS 텔레메트리는 TOF만 포함한다. MPU-9250/AK8963과
BNO055는 아직 운영 프로토콜과 이 펌웨어에 구현하지 않았다. 다리
액추에이터와 limit 입력도 핀·극성이 확정되지 않아 동작시키지 않는다.

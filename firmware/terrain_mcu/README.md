# Terrain MCU 펌웨어

Arduino Uno가 TOF-10120과 BE-220 GPS를 읽고 프로토콜 v2 텔레메트리로
Raspberry Pi에 전송한다. 결과는 `/terrain/tof`, `/terrain/status`,
`/gps/fix`, `/gps/speed`, `/diagnostics`에서 확인한다.

## 핀맵

| 기능 | Uno 핀 |
|---|---:|
| I2C SDA | A4 |
| I2C SCL | A5 |
| BE-220 TX → Uno RX | D8 |
| BE-220 RX ← Uno TX | D9, 설정할 때만 필요 |
| 센서 공통 GND | GND |

Uno에서 AltSoftSerial을 안정적으로 사용하기 위해 BE-220은 먼저 USB-TTL
어댑터와 제조사 설정 도구로 `9600 baud`에 맞춘다. 정상 수신만 할 때는
BE-220의 TX, 전원, GND만 연결하면 되며 D9는 연결하지 않아도 된다. 전원과
UART 논리레벨은 사용 중인 BE-220 보드 리비전 사양을 확인한다.

TOF 유효 범위는 100~2000 mm이다. 필터 거리와 느린 기준 거리의 차이가
60 mm를 넘으면 후보가 되고, 10 mm 이상의 상승이 4회 연속되면 단차로
판정한다. LED 대신 모든 상태를 직렬 텔레메트리와 ROS 토픽으로만 보낸다.

```bash
bash scripts/install_arduino_libraries.sh
arduino-cli compile --fqbn arduino:avr:uno firmware/terrain_mcu
arduino-cli upload --fqbn arduino:avr:uno -p /dev/safestride-terrain \
  firmware/terrain_mcu
```

현재 운영 펌웨어의 ROS 텔레메트리는 TOF와 GPS를 포함한다. MPU-9250/AK8963과
BNO055는 아직 운영 프로토콜과 이 펌웨어에 구현하지 않았다. 다리
액추에이터와 limit 입력도 핀·극성이 확정되지 않아 동작시키지 않는다.

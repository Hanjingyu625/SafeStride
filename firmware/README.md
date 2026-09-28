# SafeStride firmware pin map

Raspberry Pi와 Drive/Terrain Arduino Uno는 각각 USB로 연결한다. D0/D1은
USB serial용으로 예약하고 다른 장치를 연결하지 않는다.

## SafeStride MCU (Drive Uno)

| UNO 핀 | 연결 대상 |
|---|---|
| D0/D1 | 미연결, Raspberry Pi USB serial용 예약 |
| D5 | SZH-GNP521 PWM |
| D6 | SZH-GNP521 IN1 |
| D8 | SZH-GNP521 IN2 |
| A1 | 오른쪽 압력센서 분압 출력 |
| A2 | 왼쪽 압력센서 분압 출력 |
| D2 (INT0) | 왼쪽 A3141/A3144 디지털 홀센서 OUT, 5V로 풀업 |
| 5V | A314x VCC, 좌우 압력센서 분압회로 전원 |
| GND | 홀센서, 압력센서, SZH-GNP521 COM 공통 GND |

압력센서는 각각 FSR과 330 Ω 저항으로 분압회로를 구성하며, 운영
임계값은 좌우 모두 ADC 35이다. A314x는 왼쪽 휠에만 설치되어
있고 HIGH→LOW 전환을 회전당 12 pulse로 센다. OUT은 기존 A3에서 D2로 옮긴다.
OUT-5V 사이 4.7kΩ 풀업과 VCC-GND 사이 0.1µF 바이패스를 센서 가까이에 연결한다.
단극성 센서이므로 자석 12개가 같은 감지 극성을 향하고, 한 바퀴에 실제 12번 감지되어야 한다.
SZH-GNP521 하나의 OUT1/OUT2에 모터
두 개가 같은 출력으로 연결되며, 드라이버의 5VO는 Uno에 연결하지
않는다.

## Terrain MCU (Terrain Uno)

LCD 최초 설정 및 전원 배선은 [디스플레이 안내](../docs/DISPLAY_KO.md)를 따른다.

| UNO 핀 | 연결 대상 |
|---|---|
| D0/D1 | 미연결, Raspberry Pi USB serial용 예약 |
| A0 | CdS와 10kΩ 분압 중간점 (5V–CdS–A0–10kΩ–GND) |
| D4 | 1채널 5V 릴레이 IN, HIGH=ON 설정, COM–NO 사용 (Terrain README 참고) |
| A4 | TOF-10120 SDA, GY-521 MPU6050 SDA |
| A5 | TOF-10120 SCL, GY-521 MPU6050 SCL |
| D9 (TX) | ezHMI TTL DIN/RX |
| D8 (RX) | ezHMI TTL DOUT/TX, ACK 수신 |
| 5V | TOF-10120, GY-521 MPU6050 VCC |
| GND | TOF-10120, GY-521 MPU6050 공통 GND |

TOF-10120과 MPU6050은 같은 I2C 버스를 공유한다. TOF-10120은 `0x52`,
MPU6050은 AD0 상태에 따라 `0x68` 또는 `0x69`를 사용한다. BE-220 GPS는
Terrain Uno의 D8/D9에 연결하지 않고 Raspberry Pi GPIO UART
`/dev/serial0`으로 직접 수신한다.

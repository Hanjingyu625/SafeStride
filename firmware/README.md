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
| A3 | 왼쪽 WSH135 홀센서 OUT |
| 5V | WSH135 VDD, 좌우 압력센서 분압회로 전원 |
| GND | 홀센서, 압력센서, SZH-GNP521 COM 공통 GND |

압력센서는 각각 FSR과 330 Ω 저항으로 분압회로를 구성하며, 운영
임계값은 좌우 모두 ADC 80이다. WSH135는 왼쪽 휠에만 설치되어
있고 무자계 아날로그 기준값과 히스테리시스로 회전당 6 pulse를 센다.
SZH-GNP521 하나의 OUT1/OUT2에 모터
두 개가 같은 출력으로 연결되며, 드라이버의 5VO는 Uno에 연결하지
않는다.

## Terrain MCU (Terrain Uno)

| UNO 핀 | 연결 대상 |
|---|---|
| D0/D1 | 미연결, Raspberry Pi USB serial용 예약 |
| A4 | TOF-10120 SDA, GY-521 MPU6050 SDA |
| A5 | TOF-10120 SCL, GY-521 MPU6050 SCL |
| 5V | TOF-10120, GY-521 MPU6050 VCC |
| GND | TOF-10120, GY-521 MPU6050 공통 GND |

TOF-10120과 MPU6050은 같은 I2C 버스를 공유한다. TOF-10120은 `0x52`,
MPU6050은 AD0 상태에 따라 `0x68` 또는 `0x69`를 사용한다. BE-220 GPS는
Terrain Uno의 D8/D9에 연결하지 않고 Raspberry Pi GPIO UART
`/dev/serial0`으로 직접 수신한다.

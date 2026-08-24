# SafeStride 하드웨어 연결표

## 보드 역할

| 장치 | 담당 보드 | 연결 |
|---|---|---|
| 동일 정격 모터 2개 | Drive Uno의 단일 SZH-GNP521 | 하나의 공통 출력 부하 |
| 휠 엔코더(모델 미정) | Drive Uno | D2, D3 예약 |
| 좌우 압력센서 | Drive Uno | A1, A2 전압분배기 |
| E-stop | 미구현 | D12 예약, 현재 입력으로 설정하지 않음 |
| TOF-10120 | Terrain Uno | I2C A4/A5, `0x52` |
| BE-220 GPS | Terrain Uno | GPS TX → D8, 선택적 GPS RX ← D9 |
| 카메라 | Raspberry Pi | CSI/USB |

## Drive Uno 핀맵

| Uno | 기능 |
|---:|---|
| D2 | `ENCODER_INPUT_1_PIN` 예약(interrupt 가능) |
| D3 | `ENCODER_INPUT_2_PIN` 예약(interrupt 가능) |
| D5 | 단일 SZH-GNP521 PWM |
| D6 | 단일 SZH-GNP521 INA(코드의 IN1) |
| D8 | 단일 SZH-GNP521 INB(코드의 IN2) |
| A0 | 고장 핀, 현재 미사용 |
| A1/A2 | 좌우 압력센서 |
| D12 | E-stop placeholder, 현재 미구현·미사용 |
| D13 | 선택적 드라이버 fault |

엔코더 실물이 정해질 때까지 D2/D3에는 아무 장치도 연결하지 않는다. quadrature
A/B인지 좌우 개별 채널인지, 입력 전압과 pull-up 조건이 확정되기 전에는 firmware도
핀을 입력으로 설정하지 않는다. D4, D7, D9, D10은 비어 있다.

드라이버 COM과 Uno GND를 공통 연결하고 MCU reset 중에도 정지하도록 PWM 입력에
외부 풀다운을 둔다. `5VO`처럼 출력으로 표시된 단자는 Uno 5 V와 연결하지 않는다.
두 모터의 합산 정지전류와 드라이버·배터리·퓨즈·배선 허용전류를 전원 인가 전에
확인한다. E-stop 구현 전에는 시험용 물리 전원 차단 수단을 준비한다.

## Terrain Uno 핀맵

| Uno | 기능 |
|---:|---|
| A4 | I2C SDA |
| A5 | I2C SCL |
| D8 | AltSoftSerial RX, BE-220 TX 연결 |
| D9 | AltSoftSerial TX, BE-220 RX 연결(설정 시에만 선택) |
| GND | 모든 센서 공통 기준 |

## 보정 필수값

- 엔코더 모델, 출력 방식, 논리 전압과 입력 회로
- 엔코더 카운트/회전, 감속비, 방향, 휠 출력축 환산값
- 압력센서 놓음/잡음 값과 좌우 임계값
- 휠 반지름
- TOF 설치 높이와 측정 방향

엔코더와 압력 임계값을 보정하고 closed-loop 정지 조건을 검증하기 전에는 사람을
태우거나 바닥에서 모터를 구동하지 않는다.

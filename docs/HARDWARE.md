# 하드웨어 연결

## 전원

```text
12 V battery -> motor driver
            +-> XL4015 (5 V) -> Raspberry Pi -> USB -> Uno 2대
```

FND 표시가 12 V로 일정해도 Pi 단자의 5 V 순간 강하는 별도 측정해야 한다.
모터 기동 중 XL4015 출력, 배선 전압강하와 Pi undervoltage 기록을 확인한다.
배터리 전압 측정용 분압 회로는 아직 없으므로 Arduino A5에 12 V를 직접 연결하면
안 된다.

## Drive Uno

| 핀 | 연결 |
|---:|---|
| D2 | 왼쪽 휠 Hall, `INPUT_PULLUP`, `FALLING`, 자석 6개 |
| D5/D6/D8 | 공통 모터드라이버 PWM/IN1/IN2 |
| A2/A1 | 왼쪽/오른쪽 압력센서, threshold 80 |
| A0 | 예약, 현재 미사용 |
| D12 | E-stop placeholder, currently unused |
| D13 | 선택적 driver fault, 기본 비활성 |

두 모터는 한 드라이버 출력에 연결되므로 서로 다른 속도나 제자리 회전은 지원하지
않는다. 두 모터 합산 기동·정지전류가 드라이버, 퓨즈, 배선 허용치를 넘지 않는지
확인한다.

## Terrain Uno

| 핀 | 연결 |
|---:|---|
| A4/A5 | TOF-10120(`0x52`)와 GY-521 MPU6050(`0x68`/`0x69`) SDA/SCL |
| GND | 모든 센서와 공통 GND |

TOF는 아래쪽을 향해 약 25 cm 높이에 고정하고 장착부 흔들림이 거리 변화로
들어오지 않게 한다.

## Raspberry Pi GPS

BE-220은 Pi GPIO UART를 통해 Raspberry Pi에 직접 연결하고, 운영 장치 이름은
`/dev/serial0` 또는 `/dev/ttyS0`, 기본 baudrate는 115200이다. 실행 스크립트가
두 경로를 자동 선택하며 필요하면 `SAFESTRIDE_GPS_PORT`로 지정한다. GPS와 Pi의
GND를 공유하고 TTL 전압 레벨이 Pi 입력 허용 범위와 맞는지 확인한다.

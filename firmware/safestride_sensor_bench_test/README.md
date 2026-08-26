# SafeStride MCU 센서 단독 테스트

SafeStride/Drive Arduino Uno에서 좌우 홀센서, 압력센서와 E-stop을 터미널로
확인한다. 상태 LED는 사용하지 않으며 단일 모터드라이버 출력은 테스트 내내
PWM=0, IN1=LOW, IN2=LOW로 유지한다. 모터 배터리는 분리한다.

## 핀맵

| Uno 핀 | 연결 대상 | 동작 |
|---|---|---|
| D2 | 왼쪽 홀센서 디지털 출력 | `INPUT_PULLUP`, falling edge 카운트 |
| D3 | 오른쪽 홀센서 디지털 출력 | `INPUT_PULLUP`, falling edge 카운트 |
| D5/D6/D8 | 단일 드라이버 PWM/IN1/IN2 | 항상 `0/LOW/LOW` |
| A0 | 왼쪽 압력센서 | 아날로그 입력 |
| A1 | 오른쪽 압력센서 | 아날로그 입력 |
| A2 | NC E-stop 접점 | `INPUT_PULLUP`, HIGH=정지 |
| D13 | 드라이버 Fault 후보 | 기본 비활성 |
| 5V/GND | 센서 전원/공통 기준 | 실제 모듈 사양 확인 |

D4, D7, D9, D10, D12와 상태 LED 핀은 사용하지 않는다. 홀센서가 push-pull
active-high 출력이라면 운영 설정의 `HALL_ACTIVE_LEVEL`과 입력 회로를 실제
모듈에 맞춰 변경해야 한다.

## 업로드

```powershell
arduino-cli compile --fqbn arduino:avr:uno firmware/safestride_sensor_bench_test
arduino-cli upload --fqbn arduino:avr:uno -p COM3 firmware/safestride_sensor_bench_test
```

시리얼 모니터는 `115200 baud`, 줄바꿈은 `Newline`으로 설정한다.

## 테스트 순서

1. `STREAM OFF`, `ZERO`를 차례로 입력한다.
2. 왼쪽 휠을 정확히 10회 돌린 후 `STATUS`를 입력한다.
3. `hall_l_pulses / 10`이 왼쪽 휠 1회전당 펄스 수다. 오른쪽도 같은 방법으로
   측정한다. 양쪽 값이 달라지면 센서 간격, 자석 수 또는 노이즈를 점검한다.
4. 천천히/빠르게 돌렸을 때 `hall_*_hz`, `hall_*_rpm`과
   `hall_*_speed_mps`가 증가하는지 확인한다. 선속도는 스케치의
   `WHEEL_RADIUS_M`을 사용하므로 실제 바퀴 반지름으로 수정한다.
5. 압력센서를 놓고 누르며 `pressure_*_raw`, `filtered`, `present`를 확인한다.
6. 양손을 누르면 `deadman=1`, 한쪽을 놓으면 `deadman=0`인지 확인한다.
7. E-stop을 누르거나 배선을 분리하면 `estop=1`인지 확인한다.

```text
STATUS
ZERO
STREAM ON
STREAM OFF
HELP
```

측정한 펄스 수를 운영 `config.h`, 이 테스트벤치의
`HALL_PULSES_PER_WHEEL_REV`, 두 ROS YAML의
`base.hall_pulses_per_revolution`에 동일하게 넣는다. 속도와 무펄스 fault를
들어 올린 휠에서 검증한 뒤 운영 설정과 벤치의
`HALL_CALIBRATED=true`로 변경한다.

## 단일 자석의 제어 한계

RPM은 `60 * pulse_hz / pulses_per_revolution`, 선속도는
`2*pi*wheel_radius * pulse_hz / pulses_per_revolution`으로 계산한다. 첫
펄스는 기준 시각만 만들기 때문에 두 번째 펄스부터 속도가 표시된다. 마지막
펄스가 `HALL_ZERO_TIMEOUT_US`보다 오래되면 `hall_*_stopped=1`이고 속도는
0으로 표시된다.

바퀴당 자석이 1개뿐이면 저속에서 펄스 간격이 매우 길다. 운영 설정의 stall
감시 시작 속도 0.3 rad/s에서는 한 바퀴가 약 20.9초 걸리므로 1.5초 stall
제한 안에 정상 펄스가 도착하지 않는다. 해당 조건을 구분하려면 이론상 최소
14 pulse/rev가 필요하다. 따라서 `HALL_CALIBRATED`를 활성화하기 전에 자석
수를 늘리거나 기어박스/모터축의 고해상도 출력을 사용해야 한다. 자석 수를
늘리면 `HALL_MIN_PULSE_INTERVAL_US`도 최대 RPM에 맞춰 다시 계산한다.

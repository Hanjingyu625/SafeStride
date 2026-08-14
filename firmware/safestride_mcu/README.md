# SafeStride Drive MCU 펌웨어

Arduino Uno가 단일 SZH-GNP521을 통해 두 모터에 공통 속도 명령을 내리고,
좌우 홀센서·압력센서를 독립 감시한다. E-stop은 현재 미구현이며 항상 정상으로
보고한다. 직렬 포트는 텍스트가 아닌
[프로토콜 v2](../../PROTOCOL.md)를 사용하므로 운영 중에는 ROS 토픽으로
상태를 확인한다.

## 핀맵

| 기능 | Uno 핀 |
|---|---:|
| 왼쪽 홀센서 출력 | D2 (interrupt) |
| 오른쪽 홀센서 출력 | D3 (interrupt) |
| 단일 드라이버 PWM / INA(IN1) / INB(IN2) | D5 / D6 / D8 |
| 왼쪽/오른쪽 압력센서 | A0 / A1 |
| E-stop | A2 예약, 현재 미구현·미사용 |
| 드라이버 fault(기본 비활성) | D13 |

D4, D7, D9, D10, D12는 현재 사용하지 않는다. 상태 LED 출력도 사용하지
않는다.

## 현재 자석 펄스 벤치 모드

바퀴가 없는 회로 시험을 위해 `config.h`의 `MAGNET_BENCH_MODE=true`가 설정되어
있다. ROS에서 최신 직진 속도 명령을 발행하고 명시적으로 enable한 뒤 D2 또는
D3 홀센서에 자석을 통과시키면 PWM 60으로 두 모터가 750 ms 동안 함께 구동된다.
펄스를 반복하면 마지막 펄스 기준으로 구동 시간이 연장된다. 이 모드는 홀 보정,
압력 dead-man, 정지 대기, Hall stall/overspeed fault를 우회하지만 serial session과
command watchdog은 유지한다. 정상 운용 전에는 펌웨어의
`MAGNET_BENCH_MODE=false`와 두 ROS YAML의 `allow_magnet_bench_mode: false`를
함께 적용해야 한다.

## 필수 보정

1. `HALL_CALIBRATED=false`인 운영 펌웨어를 업로드하고 ROS bridge를 실행한다.
2. `/wheel/hall`의 시작 펄스 값을 기록한 뒤 각 휠을 정확히 10회 회전한다.
3. 종료값과 시작값 차이의 절댓값을 10으로 나눠 실제 회전당 펄스 수를 얻는다.
4. `config.h`와 두 ROS YAML의 회전당 펄스 수를 같은 값으로 수정한다. 두 센서가 다른
   값을 내면 기구 또는 센서 설치를 먼저 수정한다.
5. 속도와 fault 임계값을 들어 올린 휠에서 검증한 뒤
   `HALL_CALIBRATED=true`로 변경한다. 정상 모드에서는 `false`이면 MCU와 ROS
   브리지가 모두 모터 활성화를 거부한다.
6. 운영 펌웨어를 다시 업로드하고 `/handle/pressure` 로그로 좌우 압력 임계값도 보정한다.

단일출력 홀센서는 회전 방향을 직접 측정하지 못하므로 부호는 드라이버 명령
방향을 따른다. 외력으로 역회전하는 상황의 signed 위치는 보장되지 않는다.

## 빌드

```bash
arduino-cli compile --fqbn arduino:avr:uno firmware/safestride_mcu
arduino-cli upload --fqbn arduino:avr:uno -p /dev/safestride-drive \
  firmware/safestride_mcu
```

두 모터를 하나의 드라이버 출력에 연결하는 설계는 코드만으로 안전성을
보장할 수 없다. 동일 정격 모터인지, 병렬/극성 연결이 맞는지, 두 모터의
합산 정지전류를 드라이버·배터리·퓨즈·배선이 견디는지 전원 인가 전에 확인한다.
현재 E-stop 입력은 동작하지 않으므로 시험 중에는 별도의 물리 전원 차단 수단을
준비해야 한다. 향후 E-stop을 구현할 때는 MCU 입력뿐 아니라 구동 전원 또는
드라이버 enable을 하드웨어로 차단해야 한다.

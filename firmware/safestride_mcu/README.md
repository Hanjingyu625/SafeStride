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
| 왼쪽/오른쪽 압력센서 | A1 / A2 |
| E-stop | 현재 미구현·미사용, 핀 미할당 |
| 드라이버 fault(기본 비활성) | D13 |

D4, D7, D9, D10은 현재 사용하지 않는다. D12는 E-stop placeholder지만
E-stop이 미구현이라 입력으로 설정되지 않는다. 상태 LED 출력도 사용하지 않는다.

## 현재 홀 피드백 제어

자석을 한 번 대면 고정 시간 동안 모터를 켜던 임시 벤치 모드는 제거했다. 현재는
ROS 또는 프로토콜의 목표 속도를 받은 뒤 D5에 PWM을 출력하고, D2/D3에서 측정한
좌우 바퀴 속도의 평균을 PID 입력으로 사용한다. 모터의 기동 데드존 때문에 출력이
필요할 때는 PWM 90부터 시작하며, 측정 속도가 목표에 도달하거나 초과하면 모터를
역구동하지 않고 coast한다. 세션·명령 watchdog과 좌우 홀센서 stall/overspeed fault는
항상 동작한다.

현재 하드웨어 시험을 위해 `REQUIRE_DEADMAN=false`라서 dead-man 상태는 항상
활성으로 보고하지만 압력센서 값은 계속 전송한다. 실제 주행 전에는 압력 임계값을
보정한 뒤 `REQUIRE_DEADMAN=true`로 되돌려야 한다.

## 필수 보정

1. 현재 설정은 각 휠에 자석 1개, 즉 `HALL_PULSES_PER_WHEEL_REV=1`을 가정한다.
2. `/wheel/hall`의 시작 펄스 값을 기록한 뒤 각 휠을 정확히 10회 회전한다.
3. 종료값과 시작값 차이의 절댓값을 10으로 나눠 실제 회전당 펄스 수를 확인한다.
4. 값이 1이 아니면 `config.h`와 두 ROS YAML의 회전당 펄스 수를 같은 값으로 수정한다. 두 센서가 다른
   값을 내면 기구 또는 센서 설치를 먼저 수정한다.
5. 속도와 fault 임계값을 들어 올린 휠에서 검증한다. `HALL_CALIBRATED=false`이면
   MCU와 ROS 브리지가 모두 모터 활성화를 거부한다.
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

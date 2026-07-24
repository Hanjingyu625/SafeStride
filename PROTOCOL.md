# SafeStride 직렬 통신 프로토콜 v1

이 문서는 `safestride_bridge`와 `firmware/safestride_mcu`가 공통으로 따라야
하는 표준 규격이다.

## 전송 방식과 프레이밍

- 기본 전송 방식: USB CDC 직렬 통신, 115200 baud, 8-N-1
- 바이트 순서: 리틀 엔디언
- 프레임 경계: 0 바이트(`0x00`)
- 인코딩: 전체 원시 프레임에 COBS 적용
- 체크섬: CRC-16/CCITT-FALSE
  - 다항식 `0x1021`
  - 초기값 `0xffff`
  - 입력 및 출력 비트 반전 없음
  - 최종 XOR `0x0000`
- 디코딩된 프레임의 최대 크기: 128바이트

전송되는 데이터 형식은 다음과 같다.

```text
COBS(header || payload || crc16_le) || 0x00
```

CRC는 `header || payload`를 대상으로 계산한다. COBS 프레임이 잘못되었거나,
길이가 유효하지 않거나, 지원하지 않는 버전이거나, 프레임 크기가 제한을
초과하거나, CRC가 일치하지 않으면 해당 프레임을 폐기한다. 프레임을
폐기했을 때 모터 명령 watchdog의 제한 시간을 갱신해서는 안 된다.

## 공통 헤더

패킹된 헤더 형식은 `<BBBBHHII`이며, 정확히 16바이트이다.

| 오프셋 | 자료형 | 필드 |
|---:|---|---|
| 0 | `uint8` | 프로토콜 버전, 현재 `1` |
| 1 | `uint8` | 메시지 유형 |
| 2 | `uint8` | 플래그, 현재 `0` |
| 3 | `uint8` | 예약 필드, 반드시 `0` |
| 4 | `uint16` | 메시지 시퀀스 |
| 6 | `uint16` | payload 길이 |
| 8 | `uint32` | session ID |
| 12 | `uint32` | 송신자의 단조 증가 시간, 단위는 밀리초 |

여러 바이트로 구성된 정수는 자료형의 범위를 넘으면 자연스럽게
wraparound된다. 시퀀스를 비교할 때는 `uint16` wraparound를 고려해야 한다.

## 메시지 유형

### `HELLO` — `0x01`, MCU에서 host로

payload 형식은 `<II`이다.

| 자료형 | 필드 |
|---|---|
| `uint32` | MCU boot ID |
| `uint32` | capability bitmap |

`HELLO`는 MCU가 부팅될 때마다 session ID를 0으로 설정하여 전송하며,
session이 성립할 때까지 주기적으로 전송한다. boot ID는 부팅할 때 변경되며,
reset 이전에 버퍼에 남아 있던 명령을 거부하는 데 사용한다.

capability bitmap v1:

| 비트 | 의미 |
|---:|---|
| 0 | 바퀴 encoder 2개 |
| 1 | 거리 측정 필드 2개 |
| 2 | battery 필드 |
| 3 | motor current 필드 2개 |
| 4 | dead-man 입력 |
| 5 | E-stop 입력 |

### `SESSION_START` — `0x02`, host에서 MCU로

payload 형식은 `<I`이다.

| 자료형 | 필드 |
|---|---|
| `uint32` | MCU에서 예상하는 boot ID |

host는 0이 아닌 임의의 session ID를 생성하여 공통 헤더에 넣는다. MCU는
payload의 boot ID가 현재 boot ID와 일치하고, 활성 session이 없으며, MCU가
`HELLO`를 보낸 상태일 때만 해당 session을 수락한다. session을 수락한 뒤의
모터 상태는 항상 `DISARMED`이다.

### `COMMAND` — `0x10`, host에서 MCU로

payload 형식은 `<iiHBB`이다.

| 자료형 | 필드 |
|---|---|
| `int32` | 왼쪽 바퀴 목표 속도, 밀리라디안/초 |
| `int32` | 오른쪽 바퀴 목표 속도, 밀리라디안/초 |
| `uint16` | 명령 time-to-live, 밀리초 |
| `uint8` | enable 요청(`0` 또는 `1`) |
| `uint8` | 예약 필드, 반드시 `0` |

명령은 다음 조건을 모두 만족할 때만 수락한다.

- version, type, length, CRC가 유효하다.
- session ID가 활성 session과 일치한다.
- sequence가 마지막으로 수락된 명령보다 최신이다.
- TTL이 0이 아니며 firmware에 설정된 최댓값 이내이다.
- 모든 목표값이 설정된 절댓값 제한 이내이다.
- E-stop, dead-man, local fault 조건이 요청된 상태를 허용한다.

수락된 최신 명령만 watchdog 제한 시간을 갱신한다. `DISARMED` 상태에서
enable하려면, 설정된 dwell 시간 동안 바퀴 속도 feedback이 정지 상태여야
하며 neutral 목표값을 여러 번 받아야 한다. MCU가 `ARMED` 상태를 확인한
후에도 host bridge는 더 최신의 neutral supervised command를 받을 때까지
출력을 0으로 유지한다. reset, E-stop, watchdog timeout, session 변경 또는
critical fault가 발생하면 명시적인 새 arm 절차가 필요하다.

MCU의 command watchdog 제한 시간이 만료되면 대기 중인 직렬 입력을
해석하기 전에 전체 session을 무효화한다. 이후 MCU는 새로운 `HELLO`를
보내며, 만료된 session의 명령으로는 정지 상태를 해제하거나 다시 arm할 수
없다.

### `TELEMETRY` — `0x20`, MCU에서 host로

payload 형식은 `<iiiiHHHhhHHH`이다.

| 자료형 | 필드 |
|---|---|
| `int32` | 왼쪽 encoder count |
| `int32` | 오른쪽 encoder count |
| `int32` | 측정된 왼쪽 속도, 밀리라디안/초 |
| `int32` | 측정된 오른쪽 속도, 밀리라디안/초 |
| `uint16` | 왼쪽 거리, 밀리미터. `0xffff`는 유효하지 않음을 의미 |
| `uint16` | 오른쪽 거리, 밀리미터. `0xffff`는 유효하지 않음을 의미 |
| `uint16` | battery voltage, 밀리볼트. `0xffff`는 유효하지 않음을 의미 |
| `int16` | 왼쪽 motor current, 밀리암페어. `INT16_MIN`은 유효하지 않음을 의미 |
| `int16` | 오른쪽 motor current, 밀리암페어. `INT16_MIN`은 유효하지 않음을 의미 |
| `uint16` | status bitmap |
| `uint16` | fault bitmap |
| `uint16` | 마지막으로 수락된 command sequence |

status bitmap:

| 비트 | 의미 |
|---:|---|
| 0 | session 활성 상태 |
| 1 | motor output enable/armed 상태 |
| 2 | dead-man 활성 상태 |
| 3 | E-stop 활성 상태 |
| 4 | command watchdog timeout 발생 |
| 5 | 현재 session에서 유효한 명령을 한 번 이상 수신함 |
| 8..10 | firmware state enum |

firmware state enum:

| 값 | 상태 |
|---:|---|
| 0 | `BOOT` |
| 1 | `DISARMED` |
| 2 | `ARMED` |
| 3 | `SAFE_STOP` |
| 4 | `ESTOP` |
| 5 | `FAULT` |

fault bitmap의 값은 하드웨어 구성에 따라 달라진다. 각 비트는 문서화해야
하며, 한 번 할당한 뒤에는 의미가 바뀌지 않도록 유지해야 한다. 활성 상태의
critical fault가 있으면 arm할 수 없다. 폐기된 프레임 수와 같은 진단용
counter를 영구적으로 latch되는 critical fault로 나타내서는 안 된다.

## Host 명령 변환

바퀴 반지름이 `r`, 왼쪽과 오른쪽 바퀴 사이의 거리가 `L`, 요청한 선속도가
`v`, 요청한 yaw 속도가 `w`일 때 host는 다음과 같이 계산한다.

```text
left_rad_s  = (v - w * L / 2) / r
right_rad_s = (v + w * L / 2) / r
```

계산 결과가 허용 범위 안에 있는지 검사한 뒤 정수 밀리라디안/초 단위로
변환한다. 부동소수점 값, compiler-native C struct, NaN 및 host padding은
통신선으로 전송하지 않는다.

## 기본 타이밍 설정

다음 값은 초기 구동을 위한 기본값이며, 모든 시스템에 공통으로 적용되는
안전 요구사항은 아니다.

| 기능 | 기본값 |
|---|---:|
| motor PID | 200 Hz |
| command stream | 50 Hz |
| telemetry | 100 Hz |
| command TTL | 200 ms |
| host telemetry-stale threshold | 300 ms |

firmware는 명령의 경과 시간을 계산할 때 자체 단조 증가 수신 시간을
사용한다. ROS wall time은 안전용 clock이 아니다. DDS QoS, process
supervision 및 Linux service restart는 MCU watchdog이나 hardware E-stop을
대체하지 않는다.

# SafeStride 직렬 통신 프로토콜 v5

Drive/Terrain Uno와 Raspberry Pi가 사용하는 115200 baud, little-endian,
COBS+CRC16-CCITT-FALSE 프로토콜이다. 호환성 값은 version `5`, schema
`0x0501`, release `20260906`이다.

공통 16-byte 헤더는 `<BBBBHHII>`이며 version, type, flags, reserved,
sequence, payload length, session ID, MCU timestamp 순서다. flags와 reserved는
0이어야 한다. CRC/길이/session/sequence 검사를 통과하지 못한 프레임은 버린다.

## 메시지

| type | 방향 | payload |
|---:|---|---|
| `0x01 HELLO` | MCU→Pi | `<IIBBHI>` boot, capabilities, role, version, schema, release |
| `0x02 SESSION_START` | Pi→MCU | `<IBBHI>` 기대 boot/role/version/schema/release |
| `0x10 COMMAND` | Pi→Drive | `<iHBBhBB>`, 12 bytes; 목표 mrad/s, TTL, enable, reserved, slope FF, cap, mode |
| `0x20 TELEMETRY` | Drive→Pi | `<iiiiHHHhhHHHHHHHBBhhhIBB>`, 54 bytes |
| `0x21 TERRAIN_TELEMETRY` | Terrain→Pi | 45 bytes, 아래 표 |

Capability bit 0은 왼쪽 단일 홀센서, 4는 dead-man, 6은 압력, 8은 TOF,
9는 MPU6050이다. E-stop bit 5는 현재 광고하지 않는다.

Drive telemetry의 왼쪽/오른쪽 pulse와 velocity 필드는 wire 호환을 위해 유지한다.
실제 입력은 왼쪽 A3 WSH135 하나이며 오른쪽 필드는 같은 값을 복제한다. 배터리 분압과
전류센서가 비활성이면 각각 `0xffff`, `INT16_MIN` sentinel을 보낸다.

## Terrain telemetry

활성 필드 형식은 `<HBBHHhhhhhhhhhhBH>`이고, protocol v4의 45-byte wire
호환성을 위해 뒤의 14 bytes는 0으로 채운 예약 영역이다. GPS는 Raspberry Pi가
직접 수신하므로 Terrain telemetry에 포함하지 않는다.

| offset | type | 내용 |
|---:|---|---|
| 0 | `uint16` | TOF raw mm |
| 2 | `uint8` | TOF valid |
| 3 | `uint8` | 0 normal, 1 raised candidate, 2 drop candidate, 3 raised, 4 drop, 5 invalid |
| 4, 6 | `uint16` | EMA filtered mm, adaptive reference mm |
| 8, 10 | `int16` | reference error mm, per-frame change mm |
| 12..16 | `int16` ×3 | MPU6050 acceleration, mg |
| 18..22 | `int16` ×3 | MPU6050 angular velocity, mrad/s |
| 24, 26 | `int16` | roll, pitch mrad |
| 28 | `uint8` | MPU valid |
| 29 | `uint16` | fault bits: bit0 TOF, bit1 MPU |
| 31..44 | `uint8` ×14 | 예약 영역, 항상 0 |

Drive Uno는 session, 최신 command TTL, Hall 보정, 양손 압력, fault 및 명시적
enable을 모두 만족할 때만 PWM을 허용한다. 단차 확정 시 ROS가 한 번 0 명령을
발행한 뒤 명령 송신을 중단하므로 Drive command watchdog도 안전 정지한다.

## v5 속도제어 계약

v4 COMMAND는 수용하지 않는다. Drive/Terrain Uno와 Pi를 함께 갱신한다.
`/drive/command`의 `DriveCommand`는 목표속도·경사 FF·상한·BRAKE mode를 한 메시지로
전달한다. `/cmd_vel_safe`는 감독된 속도 관찰용 `TwistStamped`로 계속 발행한다.
bridge는 `/drive/command`만 정상 주행 입력으로 구독하며 ROS stamp가 미래이거나
0.5초보다 오래된 메시지를 거부한다. 수신 후에도 monotonic timeout 0.5초를 적용한다.
MCU는 기존 CRC/session/단조 sequence 및 TTL 20~250ms 검사를 유지한다.

| COMMAND offset | 형식 | 의미·허용 범위 |
|---:|---|---|
| 0 | int32 | 공통 목표 mrad/s, -3000~3000 |
| 4 | uint16 | TTL ms, 운영 기본 200 |
| 6 | uint8 | enable 0/1 |
| 7 | uint8 | reserved, 반드시 0 |
| 8 | int16 | slope FF PWM counts, -60~30 |
| 10 | uint8 | drive PWM cap, 0~100 |
| 11 | uint8 | 0 DRIVE / 1 BRAKE |

BRAKE에서는 target=0, slope FF=0이어야 한다. enable=1인 BRAKE는 링크를 유지하면서
`IN1=IN2=LOW` 상태를 요청한다. 경사 BRAKE 때문에 새 fault나 수동 재시작 잠금을
만들지 않으며 최신 DRIVE 명령으로 자동 복귀한다. enable=0·기존 fault·watchdog
처리는 별개로 유지된다. 음수 target은 후진 명령이며 제동 요청이 아니다.

기존 Drive telemetry 0..41 필드는 그대로 두고 다음 필드를 추가했다.

| offset | 형식 | 내용 |
|---:|---|---|
| 42 / 44 / 46 | int16 ×3 | feed-forward(경사 포함), feedback(소수부 절삭), 실제 적용 PWM count |
| 48 | uint32 | 마지막 Hall pulse age, µs; pulse 없음은 0xffffffff |
| 52 | uint8 | bit0 speed valid, bit1 직전 성공 telemetry 이후 새 pulse; 나머지 0 |
| 53 | uint8 | 0 DRIVE / 1 BRAKE 출력 상태 |

`WalkerStatus`에 위 정보와 `measured_speed_m_s`, `direction_valid=false`를 발행한다.
속도 부호는 명령으로 추정한다. telemetry 수신 빈도는 새 Hall 측정 빈도가 아니다.

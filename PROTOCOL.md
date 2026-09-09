# SafeStride 직렬 통신 프로토콜 v4

Drive/Terrain Uno와 Raspberry Pi가 사용하는 115200 baud, little-endian,
COBS+CRC16-CCITT-FALSE 프로토콜이다. 호환성 값은 version `4`, schema
`0x0401`, release `20260826`이다.

공통 16-byte 헤더는 `<BBBBHHII>`이며 version, type, flags, reserved,
sequence, payload length, session ID, MCU timestamp 순서다. flags와 reserved는
0이어야 한다. CRC/길이/session/sequence 검사를 통과하지 못한 프레임은 버린다.

## 메시지

| type | 방향 | payload |
|---:|---|---|
| `0x01 HELLO` | MCU→Pi | `<IIBBHI>` boot, capabilities, role, version, schema, release |
| `0x02 SESSION_START` | Pi→MCU | `<IBBHI>` 기대 boot/role/version/schema/release |
| `0x10 COMMAND` | Pi→Drive | `<iHBB>` 공통 목표 mrad/s, TTL, enable, reserved |
| `0x20 TELEMETRY` | Drive→Pi | `<iiiiHHHhhHHHHHHHBB>`, 42 bytes |
| `0x21 TERRAIN_TELEMETRY` | Terrain→Pi | 45 bytes, 아래 표 |

Capability bit 0은 왼쪽 단일 홀센서, 4는 dead-man, 6은 압력, 8은 TOF,
9는 MPU6050이다. E-stop bit 5는 현재 광고하지 않는다.

Drive telemetry의 왼쪽/오른쪽 pulse와 velocity 필드는 wire 호환을 위해 유지한다.
실제 입력은 왼쪽 D2 하나이며 오른쪽 필드는 같은 값을 복제한다. 배터리 분압과
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

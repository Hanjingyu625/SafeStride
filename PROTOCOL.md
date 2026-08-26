# SafeStride 직렬 통신 프로토콜 v3

Raspberry Pi의 `safestride_bridge`와 Drive/Terrain Uno가 공통으로 사용하는
USB 직렬 프로토콜이다. v3는 `board_role`, 스키마 ID와 펌웨어 릴리스 ID를
핸드셰이크에 포함해 오래된 펌웨어나 서로 바뀐 Drive/Terrain 포트를 명시적으로
거부한다. v1/v2 펌웨어와 v3 브리지는 호환되지 않으므로 두 Uno와 Raspberry Pi
소프트웨어를 반드시 함께 갱신해야 한다.

현재 호환성 값은 protocol `3`, schema `0x0301`, firmware release
`20260816`이다.

## 프레임

- 115200 baud, 8-N-1
- little-endian
- `COBS(header || payload || crc16_le) || 0x00`
- CRC-16/CCITT-FALSE: polynomial `0x1021`, initial value `0xffff`
- 디코딩된 최대 프레임 크기: 128 bytes

공통 헤더 형식은 `<BBBBHHII>`로 16 bytes이다.

| 오프셋 | 자료형 | 내용 |
|---:|---|---|
| 0 | `uint8` | 프로토콜 버전, `3` |
| 1 | `uint8` | 메시지 타입 |
| 2 | `uint8` | flags, 반드시 `0` |
| 3 | `uint8` | reserved, 반드시 `0` |
| 4 | `uint16` | sequence |
| 6 | `uint16` | payload 길이 |
| 8 | `uint32` | session ID |
| 12 | `uint32` | 송신 MCU의 `millis()` |

CRC 오류, 크기 오류, 잘못된 세션이나 중복·과거 sequence는 명령 watchdog을
갱신하지 않는다.

## 메시지

### `HELLO` (`0x01`, MCU → Pi)

Payload `<IIBBHI>`, 16 bytes이다.

| 자료형 | 내용 |
|---|---|
| `uint32` | `boot_id` |
| `uint32` | capabilities |
| `uint8` | board role (`1` Drive, `2` Terrain) |
| `uint8` | payload가 기대하는 protocol version (`3`) |
| `uint16` | schema ID (`0x0301`) |
| `uint32` | firmware release ID (`20260816`) |

| capability bit | 의미 |
|---:|---|
| 0 | 좌우 단일출력 홀센서 2개 |
| 1 | 전방 거리센서 2개 |
| 2 | 배터리 전압 |
| 3 | 전류 측정 2개(예약) |
| 4 | dead-man 입력 |
| 5 | E-stop 입력(현재 미구현이므로 Drive Uno가 광고하지 않음) |
| 6 | 좌우 압력센서 텔레메트리 |
| 7 | 자석 펄스 모터 벤치 모드(임시 시험 빌드) |
| 8 | Terrain TOF 텔레메트리 |
| 9 | Terrain BNO055 orientation 텔레메트리 |

### `SESSION_START` (`0x02`, Pi → MCU)

Payload `<IBBHI>`, 12 bytes이다. MCU가 보낸 `boot_id`, 기대 board role,
protocol version, schema ID, firmware release ID를 Pi가 되돌려 보낸다. MCU는
하나라도 다르면 세션을 시작하지 않는다. Pi가 0이 아닌 새 session ID를 헤더에
넣으며, 재부팅 또는 watchdog 만료 후에는 이전 세션 명령을 재사용할 수 없다.

### `COMMAND` (`0x10`, Pi → Drive Uno)

Payload `<iHBB>`, 8 bytes이다.

| 자료형 | 내용 |
|---|---|
| `int32` | 두 모터 공통 목표 속도, mrad/s |
| `uint16` | TTL, ms |
| `uint8` | enable (`0` 또는 `1`) |
| `uint8` | reserved, 반드시 `0` |

Drive Uno는 홀센서 보정, dead-man, fault, session, 정지 대기 조건을 모두
만족할 때만 enable 명령을 수락한다. E-stop은 현재 미구현이며 입력을 읽지 않고
정상 상태로 보고한다. 단일 드라이버 구조이므로 회전
목표는 존재하지 않는다. ROS 브리지는 허용치를 넘는 `angular.z` 명령을
거부하고 명시적으로 다시 활성화하기 전까지 정지 상태를 유지한다.

Capability/status bit 7이 모두 설정된 임시 자석 벤치 빌드에서는 ROS 측의
`allow_magnet_bench_mode`도 명시적으로 true일 때만 홀 보정, dead-man, 정지
대기를 우회한다. `auto_arm_magnet_bench_mode=true`이면 새 속도 명령이 들어올
때 테스트 세션만 자동 arm된다. 최신 속도 명령이 있어도 모터 출력은 D2 또는
D3의 최근 펄스가 있을 때만 고정 저출력으로 켜진다. 세션 및 command watchdog은
이 모드에서도 유지된다. status의 dead-man bit는 우회 여부와 무관하게 실제
압력센서 판정값을 계속 보고한다.

### `TELEMETRY` (`0x20`, Drive Uno → Pi)

Payload `<iiiiHHHhhHHHHHBB>`, 38 bytes이다.

| 자료형 | 내용 |
|---|---|
| `int32` | 왼쪽 홀센서 누적 signed 펄스 |
| `int32` | 오른쪽 홀센서 누적 signed 펄스 |
| `int32` | 왼쪽 측정 속도, mrad/s |
| `int32` | 오른쪽 측정 속도, mrad/s |
| `uint16` ×2 | 전방 거리, mm (`0xffff`=무효) |
| `uint16` | 배터리, mV (`0xffff`=무효) |
| `int16` ×2 | 전류, mA (`INT16_MIN`=무효) |
| `uint16` | status bitmap |
| `uint16` | fault bitmap |
| `uint16` | 마지막 수락 command sequence |
| `uint16` ×2 | 좌우 압력 ADC 값 |
| `uint8` | pressure flags |
| `uint8` | pressure alert (`0` 정상, `1` 경고, `2` 손 이탈) |

단일출력 홀센서는 자체적으로 회전 방향을 알 수 없다. 펄스 위치와 속도의
부호는 공통 드라이버 명령 방향에서 얻는다. 외력으로 역방향 이동하는 경우의
부호는 보장되지 않는다.

Status bitmap:

| bit | 의미 |
|---:|---|
| 0 | session active |
| 1 | motor armed |
| 2 | dead-man active |
| 3 | E-stop active(현재 구현에서는 항상 `0`) |
| 4 | command watchdog timeout |
| 5 | 현재 session에서 유효 명령 수신 |
| 6 | 휠 1회전당 홀 펄스 수 보정 완료 |
| 7 | 자석 펄스 모터 벤치 모드 활성 |
| 8..10 | firmware state (`BOOT=0`, `DISARMED=1`, `ARMED=2`, `SAFE_STOP=3`, `ESTOP=4`, `FAULT=5`) |

Fault bitmap의 공통 값은 `WalkerStatus.msg`와 같다. `0x0002`는 단일
모터드라이버 fault, `0x0008`과 `0x0010`은 각각 왼쪽·오른쪽 홀센서 fault다.

### `TERRAIN_TELEMETRY` (`0x21`, Terrain Uno → Pi)

Payload `<HBBHHhhhhhBBH>`, 22 bytes이다.

| 자료형 | 내용 |
|---|---|
| `uint16` | TOF 거리, mm (`0xffff`=무효) |
| `uint8` | TOF valid |
| `uint8` | alert (`0` 정상, `1` 후보, `2` 단차, `3` 무효) |
| `uint16` | 필터 거리, mm |
| `uint16` | 기준 거리, mm |
| `int16` | 기준 대비 오차, mm |
| `int16` | 직전 필터값 대비 변화, mm |
| `int16` | BNO055 heading, mrad |
| `int16` | BNO055 roll, mrad |
| `int16` | BNO055 pitch, mrad |
| `uint8` | BNO055 valid |
| `uint8` | BNO055 calibration status 원시 byte |
| `uint16` | Terrain fault bitmap |

Terrain fault bit 0은 TOF 무효, bit 1은 BNO055 무효이다. MPU-9250/AK8963은
아직 운영 펌웨어와 프로토콜에 구현되지 않았다.

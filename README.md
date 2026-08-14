# SafeStride

## Ethernet and remote ROS 2

For Raspberry Pi Ethernet setup (router DHCP or a direct Windows-to-Pi cable),
SSH remote build/run commands, and ROS 2 LAN discovery settings, see
[docs/ETHERNET.md](docs/ETHERNET.md).

SafeStride는 Raspberry Pi 4, 64비트 Ubuntu Server 24.04(Noble), ROS 2 Jazzy,
Python 3.12 환경을 대상으로 개발하는 스마트 보행기 ROS 2 워크스페이스다.

> 이 저장소는 개발용 기반 코드이며 인증된 안전 제어기가 아니다. 장치에 하중을
> 가하지 않은 상태에서 배선, 제한값, 모터 극성 및 watchdog을 검증하기 전까지
> 모든 모터 출력을 비활성화해야 한다. 사람이 보행기에 몸을 의지한 상태로 최초
> 시험을 시작해서는 안 된다.

## 시스템 구성

```text
BE-220 GPS -----> Raspberry Pi 4 <----- camera / YOLO
                         |
              ROS 2 safety supervisor
                    /           \
         USB serial               USB serial
        Drive Uno                Terrain Uno
   shared motor / Hall       TOF / MPU / BNO055 /
   pressure / Hall           step-leg actuator
```

- Drive Uno가 단일 드라이버에 함께 연결된 두 바퀴 모터의 최종 제어 권한을 가진다.
- Terrain Uno가 계단용 다리 actuator의 최종 제어 권한을 가진다.
- YOLO 판단은 보조 정보이며 허용 속도를 낮추는 방향으로만 사용한다.
- timeout, 유효하지 않은 센서값, 알 수 없는 노면 또는 serial session 단절이
  발생하면 정지하거나 속도 배율을 0으로 만든다.
- 시스템을 시작할 때 actuator를 자동으로 활성화하지 않는다.

자세한 내용은 [시스템 구조](docs/ARCHITECTURE.md),
[하드웨어](docs/HARDWARE.md), [개발 안내](docs/DEVELOPMENT.md),
[횡단보도 기능](docs/CROSSWALK.md), [로드맵](docs/ROADMAP.md)을 참고한다.

## Raspberry Pi 4 빠른 시작

```bash
git clone <repository-url> ~/SafeStride
cd ~/SafeStride
bash scripts/install_ubuntu_24_04.sh
```

설치 스크립트가 사용자의 `dialout`, `video` 그룹을 변경하면 로그아웃한 뒤 다시
로그인한다.

```bash
cd ~/SafeStride
bash scripts/build.sh
bash scripts/test.sh
```

실행하기 전에 serial placeholder를 확인된 udev 장치 식별자로 변경하고
[개발 안내](docs/DEVELOPMENT.md)에 설명된 규칙을 설치한다. 이후 다음을 실행한다.

```bash
bash scripts/run.sh
```

기본 운영 설정은 `/dev/safestride-drive`와 `/dev/safestride-terrain`을 사용한다.
실물 센서와 모터를 단계별로 확인할 때는
[ROS 하드웨어 통합 테스트 가이드](docs/ROS_HARDWARE_TEST_KO.md)를 따른다.
장착된 TOF는 지면 단차용이므로 존재하지 않는 좌우 전방 장애물 센서를 대신하지
않는다. 전방 센서를 실제로 장착하기 전까지 `require_range_sensors`는 `false`이며,
장착 후 두 토픽을 검증하면서 `true`로 바꿔야 한다.

bringup 후 센서 링크는 다음처럼 각각 한 메시지를 받아 확인한다.

```bash
ros2 topic echo /handle/pressure --once
ros2 topic echo /wheel/hall --once
ros2 topic echo /terrain/tof --once
ros2 topic echo /terrain/status --once
ros2 topic echo /walker/status --once
```

압력 원시값을 손을 뗀 상태와 잡은 상태에서 기록한 뒤
`firmware/safestride_mcu/config.h`의 좌우 polarity/threshold를 조정하고, 검증을
마친 경우에만 `PRESSURE_THRESHOLDS_CALIBRATED`를 `true`로 바꿔 다시 flash한다.

모터는 시작 시 항상 disarmed다. 현재 E-stop은 미구현이므로 별도의 물리 전원
차단 수단을 즉시 사용할 수 있고 바퀴를 지면에서 든 상태에서만 한 터미널로
아주 작은 속도 명령을 계속 발행하고, 다른 터미널에서
명시적으로 enable한다.

```bash
ros2 topic pub --rate 20 /cmd_vel geometry_msgs/msg/TwistStamped \
  "{header: {frame_id: base_link}, twist: {linear: {x: 0.03}}}"
ros2 service call /walker/set_enabled std_srvs/srv/SetBool "{data: true}"
# 시험 직후
ros2 service call /walker/set_enabled std_srvs/srv/SetBool "{data: false}"
```

## 저장소 구조

```text
src/                         ROS 2 Jazzy 패키지
firmware/safestride_mcu/     Drive Uno 펌웨어
firmware/terrain_mcu/        Terrain Uno TOF 센서 펌웨어
config/                      하드웨어 및 실행 설정
deploy/udev/                 고정 serial 장치 별칭
deploy/systemd/              화면 없는 자동 시작 서비스
scripts/                     Noble/Jazzy 설치, 빌드, 테스트 및 실행
docker/                      Jazzy 개발 이미지
data/                        외부·생성 데이터(Git에 저장하지 않음)
models/                      모델 메타데이터와 외부 관리 가중치
test/                        PC에서 실행하는 펌웨어 테스트
```

### 폴더별 역할

| 폴더 | 역할 |
|---|---|
| `.github/workflows/` | GitHub Actions에서 Ubuntu 24.04와 ROS 2 Jazzy 빌드·테스트를 자동 실행한다. |
| `config/` | Raspberry Pi 시리얼 장치, 토픽, 속도 제한과 실제 하드웨어별 설정을 보관한다. |
| `data/external/` | 외부에서 받은 원본 학습·지도·센서 데이터를 보관한다. |
| `data/generated/` | 전처리나 변환으로 생성된 데이터를 보관한다. |
| `deploy/systemd/` | Raspberry Pi 부팅 시 SafeStride를 자동 실행하기 위한 서비스 설정이다. |
| `deploy/udev/` | 두 Arduino의 USB 포트를 `/dev/safestride-drive` 같은 고정 이름으로 지정한다. |
| `docker/` | ROS 2 Jazzy 개발 환경을 재현하는 Docker 이미지 설정이다. |
| `docs/` | 전체 구조, 하드웨어, 개발 절차와 향후 작업을 설명한다. |
| `firmware/safestride_mcu/` | Drive Uno의 단일 모터 드라이버, 좌우 홀센서, deadman, watchdog 및 시리얼 프로토콜을 구현한다. E-stop은 예약 상태다. |
| `firmware/terrain_mcu/` | TOF-10120을 읽고 CRC serial telemetry로 전송한다. IMU와 step-leg 출력은 아직 비활성이다. |
| `logs/` | 주행, 센서, fault 및 ROS bag 로그를 저장할 자리다. |
| `models/` | 노면 분류용 ONNX/YOLO 모델과 관련 메타데이터를 둘 자리다. 실제 모델은 아직 없다. |
| `scripts/` | Ubuntu 설치, ROS 빌드, 테스트, 실행 및 systemd 설치를 자동화한다. |
| `src/` | ROS 2 패키지 전체가 들어 있다. |
| `test/` | Arduino 코어 로직을 PC에서 검사하는 C++ 테스트와 Arduino stub을 포함한다. |

`src/` 아래 ROS 패키지는 센서별이 아니라 책임별로 구분한다.

| ROS 패키지 | 역할과 현재 상태 |
|---|---|
| `safestride_interfaces` | `WalkerStatus`, `WheelHall`, `TerrainStatus`, `SurfaceCondition`, `HandlePressure`, `CrosswalkStatus` 메시지와 `SetLegState` 서비스를 정의한다. |
| `safestride_bridge` | Drive/Terrain Uno와 각각 USB serial로 통신하고 압력·TOF·주행 텔레메트리를 표준 ROS 토픽으로 변환한다. |
| `safestride_control` | 일반 속도 명령에 timeout, 거리, deadman, fault와 가감속 제한을 적용한다. |
| `safestride_sensors` | BE-220 NMEA를 파싱하고 `/gps/fix`, `/gps/speed`를 발행한다. |
| `safestride_navigation` | GPS 위치, 횡단보도 지도와 보행신호 잔여시간으로 v6 상태기계를 실행하고 안전감독기 앞단에 속도 요청을 발행한다. |
| `safestride_perception` | 노면 분류 결과를 보수적인 속도 배율로 바꾸는 정책이다. 카메라·YOLO 실행 노드는 아직 없다. |
| `safestride_terrain` | 지형 센서, 양손 감지, 바퀴 속도와 limit 상태를 이용해 향후 다리 전개 가능 여부를 판단하는 정책이다. |
| `safestride_bringup` | robot state publisher, Drive/Terrain serial bridge와 safety supervisor를 한 번에 실행한다. |
| `safestride_description` | 차체, 바퀴, 센서 위치와 TF 좌표계를 URDF/Xacro로 정의한다. |

Python ROS 패키지 내부의 `resource/`는 ROS 패키지 검색 등록용이고,
`test/`는 단위 테스트, `setup.py`와 `setup.cfg`는 설치 및 실행 파일 설정,
`package.xml`은 ROS 의존성 정보다.

## ROS 토픽과 서비스

### 현재 구현된 주행 코어

토픽명은 `src/safestride_bringup/config/safestride.yaml`에서 변경할 수 있다.

| 이름 | 형식 | 발행자 -> 구독자 | 역할 |
|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/msg/TwistStamped` | ROS -> Safety Supervisor | 일반 주행 속도 명령이다. |
| `/cmd_vel_safe` | `geometry_msgs/msg/TwistStamped` | Safety Supervisor -> Serial Bridge | 안전 검사를 통과해 Drive Uno로 전달되는 속도 명령이다. |
| `/wheel/hall` | `safestride_interfaces/msg/WheelHall` | Serial Bridge -> ROS | 좌우 홀센서의 누적 펄스, 추정 속도 및 보정 상태다. |
| `/joint_states` | `sensor_msgs/msg/JointState` | Serial Bridge -> ROS | 좌우 홀센서 펄스로 계산한 바퀴 위치와 속도다. |
| `/odom` | `nav_msgs/msg/Odometry` | Serial Bridge -> ROS | 홀센서로 계산한 보행기 위치와 속도다. 단일 출력 홀센서의 부호는 공통 모터 명령 방향을 따른다. |
| `/range/front_left` | `sensor_msgs/msg/Range` | Serial Bridge -> Safety Supervisor | 왼쪽 전방 거리다. 토픽은 구현됐지만 실제 센서 드라이버는 아직 없다. |
| `/range/front_right` | `sensor_msgs/msg/Range` | Serial Bridge -> Safety Supervisor | 오른쪽 전방 거리다. 토픽은 구현됐지만 실제 센서 드라이버는 아직 없다. |
| `/battery_state` | `sensor_msgs/msg/BatteryState` | Serial Bridge -> ROS | Drive Uno가 보고한 배터리 상태다. |
| `/handle/pressure` | `safestride_interfaces/msg/HandlePressure` | Drive Serial Bridge -> ROS | 좌우 FSR 원시값, 손 감지와 임계값 검증 상태다. |
| `/walker/status` | `safestride_interfaces/msg/WalkerStatus` | Serial Bridge -> Safety Supervisor | MCU 상태, deadman, watchdog과 fault를 종합한다. E-stop 필드는 현재 항상 false다. |
| `/terrain/tof` | `sensor_msgs/msg/Range` | Terrain Serial Bridge -> ROS | TOF-10120의 거리다. 무효 측정은 NaN이다. |
| `/terrain/status` | `safestride_interfaces/msg/TerrainStatus` | Terrain Serial Bridge -> ROS | TOF 유효성, 링크 age와 terrain fault다. |
| `/gps/fix` | `sensor_msgs/msg/NavSatFix` | BE-220 GPS -> Crosswalk Controller | 현재 GPS 위치다. |
| `/gps/speed` | `std_msgs/msg/Float32` | BE-220 GPS -> Crosswalk Controller | RMC 문장의 지면 속도다. |
| `/crosswalk/status` | `safestride_interfaces/msg/CrosswalkStatus` | Crosswalk Controller -> ROS | 횡단보도 상태, 거리, 신호시간, 진입 허용과 목표 속도다. |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | Bridge·Supervisor -> ROS | 통신, 센서 및 안전 상태 진단이다. |
| `/tf` | `tf2_msgs/msg/TFMessage` | Serial Bridge 등 -> ROS | 기본적으로 `odom`에서 `base_footprint`로의 좌표 변환이다. |

| 서비스 | 형식 | 역할 |
|---|---|---|
| `/walker/set_enabled` | `std_srvs/srv/SetBool` | Drive Uno 제어기의 활성화 또는 비활성화를 요청한다. |

### Terrain Uno와 센서 토픽

Drive Uno와 Terrain Uno는 각각 Raspberry Pi에 USB serial로 연결한다. 기본
bringup은 `/dev/safestride-drive`와 `/dev/safestride-terrain`을 열고 두 bridge를
함께 실행한다. 압력과 TOF 토픽은 구현되어 있고 아래 IMU·다리 항목은 후속
하드웨어 확정 전까지 비활성이다.

| 이름 | 형식 | 상태와 역할 |
|---|---|---|
| `/terrain/tof` | `sensor_msgs/msg/Range` | 구현됨: TOF-10120 거리 |
| `/terrain/status` | `safestride_interfaces/msg/TerrainStatus` | 구현됨: TOF 유효성, telemetry age와 fault |
| `/handle/pressure` | `safestride_interfaces/msg/HandlePressure` | 구현됨: 좌우 손잡이 ADC 값 및 손 감지 |
| `/terrain/imu/mpu9250` | `sensor_msgs/msg/Imu` | 미구현: MPU-9250 자세·관성 데이터 |
| `/terrain/imu/bno055` | `sensor_msgs/msg/Imu` | 미구현: BNO055 자세·관성 데이터 |
| `/perception/surface_condition` | `safestride_interfaces/msg/SurfaceCondition` | 카메라가 판단한 노면과 권장 속도 배율 |
| `/terrain/set_leg_state` | `safestride_interfaces/srv/SetLegState` | 미구현: step-leg 전개 또는 복귀 요청 서비스 |

## 현재 제한사항

- MPU-9250 및 BNO055 하드웨어 드라이버가 아직 구현되지 않았다.
- 계단용 다리의 pin map, 구동기 및 limit switch가 아직 선정되지 않았다.
- YOLO 실행 기능에는 카메라 선정, 데이터셋 및 내보낸 모델이 필요하다.
- 횡단보도 v6 로직은 ROS로 이식됐지만 원본이 참조한 `nearest_map.py`가 제공되지
  않아 시험 장소의 `itstId`를 설정하거나 횡단보도 JSON에 매핑해야 한다.
- 바퀴 치수, 홀센서의 바퀴 1회전당 펄스 수, PID 및 압력 임계값은 예시 값이므로 실제 장비에서
  측정하고 조정해야 한다.

## 팀 작업 방법

기능별 branch와 pull request를 사용한다. GitHub Actions는 Ubuntu 24.04에서
ROS 2 Jazzy를 빌드하고 테스트한다. ARM64, 카메라, serial 통신 및 실시간 성능의
최종 검증은 실제 Raspberry Pi에서 수행해야 한다.

API key, 개인 GPS 이동 경로, 카메라 원본 영상, YOLO 가중치 및 용량이 큰 원본
shapefile을 Git에 커밋하지 않는다. 자세한 내용은
[CONTRIBUTING.md](CONTRIBUTING.md)를 참고한다.

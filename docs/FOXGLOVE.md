# Foxglove 대시보드 사용법

SafeStride의 Drive MCU, Terrain MCU, GPS와 웹캠 노드 상태를 한 화면에서
확인하기 위한 읽기 전용 대시보드다. 저장소에 바로 불러올 수 있는 레이아웃을
포함한다.

- 레이아웃: `config/foxglove/safestride.json`
- Bridge 포트: TCP 8765
- 연결 방식: Foxglove WebSocket

## 대시보드 구성

| 패널 | 표시 정보 |
|---|---|
| State Transitions | Drive 상태, dead-man, 압력 alert, TOF alert/hazard, 횡단보도 상태, 노면 분류 |
| Raw Messages | `/walker/status`의 link/arm/E-stop/fault, MCU boot/session ID, CRC/frame 오류, telemetry age |
| Diagnostics | `/diagnostics`의 MCU·GPS·웹캠·지도/API 준비 상태와 오류 |
| Speed | `/wheel/hall` 기반 속도와 `/gps/speed` 비교, 단위 m/s |
| Pressure | 좌 A2·우 A1 filtered ADC와 현재 임계값 80 |
| TOF | filtered/reference 거리와 error/change, 단위 m |
| GPS Map | `/gps/fix` 위치와 최근 5분 이동 경로 |
| Inclination | MPU6050 기반 pitch/roll, 단위 degree |

홀 속도는 현재 설정된 휠 반지름 0.115 m를 사용해 다음과 같이 표시한다.

```text
speed_m_s = /wheel/hall.left_velocity_rad_s * 0.115
```

현재 하드웨어는 왼쪽 A3 WSH135 홀센서 하나만 사용한다. `/wheel/hall`의 오른쪽 속도는
왼쪽 측정값을 복제한 공통 구동계 추정치이므로 대시보드에는 왼쪽 값만 표시한다.
휠 반지름을 바꾸면 레이아웃의 `@mul(0.115)`도 같은 값으로 수정해야 한다.

## 1. Raspberry Pi 준비

최초 1회 Foxglove Bridge를 설치한다.

```bash
sudo apt update
sudo apt install ros-${ROS_DISTRO}-foxglove-bridge
```

수동 실행은 다음처럼 Foxglove를 켜서 시작한다.

```bash
cd ~/SafeStride
SAFESTRIDE_ENABLE_FOXGLOVE=true bash scripts/run.sh
```

systemd 설치본은 `/opt/safestride/deploy/systemd/safestride.env`에서 다음 값을
설정하고 재시작한다.

```text
SAFESTRIDE_ENABLE_FOXGLOVE=true
```

```bash
sudo systemctl restart safestride
sudo systemctl status safestride --no-pager
ss -ltn | grep 8765
```

Bridge는 토픽 조회만 허용한다. 서비스, 파라미터, client publish는 차단되어
Foxglove에서 모터 enable이나 `/cmd_vel` 명령을 보낼 수 없다.

## 2. PC에서 연결하고 레이아웃 불러오기

1. Foxglove Desktop 또는 Chrome의 Foxglove Web을 연다.
2. **Open connection → Foxglove WebSocket**을 선택한다.
3. Pi와 PC가 같은 네트워크인지 확인하고 `ws://PI_IP:8765`를 입력한다.
4. **Layouts → Import from file...**에서
   `config/foxglove/safestride.json`을 선택한다.
5. 레이아웃 이름을 `SafeStride Live`로 저장한다.

로컬 PC에서 Bridge도 함께 실행 중일 때만 `ws://localhost:8765`를 사용한다.

## 3. 정상 표시 기준

- `/walker/status`: `link_ok=true`, `state=2(ARMED)`는 실제 enable 뒤에만 정상이다.
  `SAFE_STOP=3`, `ESTOP=4`, `FAULT=5`는 원인을 먼저 해소한다.
- Pressure: 손을 올리면 좌 A2와 우 A1이 임계값 80 위에 있고
  `deadman=true`가 되어야 한다. calibration/임계값은 대시보드에서 변경하지 않는다.
- Hall speed: 바퀴 정지 시 0 근처, 회전 시 양의 m/s가 나타나야 한다.
- TOF: 평지에서는 filtered와 reference가 가깝고, 단차에서 error/change와
  TOF alert가 함께 변해야 한다.
- Inclination: 정지 평지에서 pitch/roll이 0도 근처여야 한다. MPU 오류 시
  `/terrain/status.mpu_valid=false`와 Diagnostics 경고를 함께 확인한다.
- GPS: 유효 fix 전에는 위도·경도가 NaN이므로 지도가 비어 있을 수 있다.
  `ros2 topic echo /gps/fix --once`에서 `status.status >= 0`이 된 뒤 확인한다.
- 웹캠 영상은 노면 추론에 사용한 프레임을 `/camera/image/compressed`로
  1 FPS 발행한다. Foxglove의 Image 패널에서 이 토픽을 선택하면 실제 카메라
  구도와 노면 처리 결과를 함께 확인할 수 있다.

## 4. 토픽이 비어 있을 때

Pi에서 먼저 실제 발행 여부를 확인한다.

```bash
ros2 topic list
ros2 topic hz /walker/status
ros2 topic hz /wheel/hall
ros2 topic hz /handle/pressure
ros2 topic hz /terrain/status
ros2 topic echo /gps/fix --once
ros2 topic echo /diagnostics --once
```

Foxglove 연결 자체가 안 되면 Pi IP와 포트를 확인한다.

```bash
hostname -I
ss -ltn | grep 8765
journalctl -u safestride -n 100 --no-pager
```

방화벽을 사용하는 경우 전체 외부망에 열지 말고 PC 주소만 허용한다.

```bash
sudo ufw allow from PC_IP to any port 8765 proto tcp
```

## 5. 실험 로그 저장과 재생

필요한 토픽만 MCAP으로 기록하면 같은 레이아웃을 오프라인 분석에도 사용할 수
있다.

```bash
ros2 bag record -s mcap \
  /walker/status /wheel/hall /handle/pressure \
  /terrain/status /terrain/tof /terrain/imu \
  /gps/fix /gps/speed /crosswalk/status \
  /perception/surface_condition /diagnostics /odom /tf /tf_static
```

기록 후 생성된 MCAP을 PC로 복사하고 Foxglove에서 **Open local file(s)**로 연 뒤
`SafeStride Live` 레이아웃을 선택한다.

Foxglove는 관측 도구일 뿐 안전 판정이나 모터 차단 권한을 대신하지 않는다.
첫 실차 시험은 바퀴를 띄우고 별도의 물리 전원 차단 수단을 준비한 상태에서
진행한다.

공식 참고:

- <https://docs.foxglove.dev/docs/getting-started/frameworks/ros2>
- <https://docs.foxglove.dev/docs/visualization/layouts>
- <https://docs.foxglove.dev/docs/visualization/panels/plot>
- <https://docs.foxglove.dev/docs/visualization/panels/map>

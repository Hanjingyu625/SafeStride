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
| Raw Messages | `/walker/status`, `/handle/pressure`, `/terrain/status` 원본 값과 유효성 플래그 |
| Diagnostics | `/diagnostics`의 MCU·GPS·웹캠·지도/API 준비 상태와 오류 |
| Speed | `/wheel/hall.left_speed_kmh`, `/walker/status.measured_speed_kmh`, 단위 km/h. GPS 곡선은 기본 비활성화 |
| Pressure | 좌 A2·우 A1 filtered ADC와 현재 임계값 40 |
| TOF | filtered/reference 거리와 error/change, 단위 m |
| USB Camera (우측 상단) | `/camera/image/compressed`의 실제 웹캠 영상, 1 FPS |
| Surface (영상 바로 아래) | 노면 종류, 신뢰도(0–1), 판정 유효성 |
| Inclination | `/terrain/status.pitch_rad`, `roll_rad` 직접 표시, 단위 radian, 자동 Y축 범위 |

홀 속도는 현재 설정된 휠 반지름 0.115 m를 사용해 다음과 같이 표시한다.

```text
speed_kmh = /wheel/hall.left_speed_kmh
```

현재 하드웨어는 왼쪽 A3 WSH135 홀센서 하나만 사용한다. `/wheel/hall`의 오른쪽 속도는
왼쪽 측정값을 복제한 공통 구동계 추정치이므로 대시보드에는 왼쪽 값만 표시한다.
휠 반지름은 bridge 설정에서 km/h로 환산할 때 적용되므로 레이아웃에서 재환산하지 않는다.

## 1. Raspberry Pi 준비

최초 1회 Foxglove Bridge를 설치한다.

```bash
sudo apt update
sudo apt install ros-${ROS_DISTRO}-foxglove-bridge
```

수동 실행은 다음처럼 Foxglove를 켜서 시작한다.

```bash
cd ~/SafeStride
SAFESTRIDE_ENABLE_FOXGLOVE=true SAFESTRIDE_ENABLE_PERCEPTION=true bash scripts/run.sh
```

systemd 설치본은 `/opt/safestride/deploy/systemd/safestride.env`에서 다음 값을
설정하고 재시작한다.

```text
SAFESTRIDE_ENABLE_FOXGLOVE=true
SAFESTRIDE_ENABLE_PERCEPTION=true
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

레이아웃 파일을 수정해도 앱에 이미 저장된 레이아웃은 자동 갱신되지 않는다.
수정된 JSON을 다시 Import해야 적용된다. 기울기는 원본 라디안 값이며
0.1745 rad가 약 10도다. GPS 속도는 필요한 경우 Speed 패널에서 켠다.
Raw Messages에도 값이 없으면 그래프 경로보다 Bridge의 토픽 노출/수신을
먼저 확인한다. 레이아웃 변경으로 Bridge 오류나 누락된 메시지 정의가
복구되는 것은 아니다.

로컬 PC에서 Bridge도 함께 실행 중일 때만 `ws://localhost:8765`를 사용한다.

## 3. 정상 표시 기준

- `/walker/status`: `link_ok=true`, `state=2(ARMED)`는 실제 enable 뒤에만 정상이다.
  `SAFE_STOP=3`, `ESTOP=4`, `FAULT=5`는 원인을 먼저 해소한다.
- Pressure: 손을 올리면 좌 A2와 우 A1이 임계값 40 위에 있고
  `deadman=true`가 되어야 한다. calibration/임계값은 대시보드에서 변경하지 않는다.
- Hall speed: 바퀴 정지 시 0 근처, 회전 시 양의 km/h가 나타나야 한다.
- TOF: 평지에서는 filtered와 reference가 가깝고, 단차에서 error/change와
  TOF alert가 함께 변해야 한다.
- Inclination: 정지 평지에서 pitch/roll이 0도 근처여야 한다. MPU 오류 시
  `/terrain/status.mpu_valid=false`와 Diagnostics 경고를 함께 확인한다.
- GPS: 유효 fix 전에는 위도·경도가 NaN이므로 지도가 비어 있을 수 있다.
  `ros2 topic echo /gps/fix --once`에서 `status.status >= 0`이 된 뒤 확인한다.
- 웹캠 영상은 노면 추론에 사용한 프레임을 `/camera/image/compressed`로
  1 FPS 발행한다. Foxglove의 Image 패널에서 이 토픽을 선택하면 실제 카메라
  구도와 노면 처리 결과를 함께 확인할 수 있다.

## 웹캠과 노면 판정

우측은 **영상 → 노면 종류·신뢰도·유효성 → Pitch/Roll** 순서다.
Map은 GPS 지도용이므로 카메라 영상에는 Image 패널을 사용한다.
신뢰도 0.8은 모델의 판단 점수 80%이며 실제 정답률을 뜻하지 않는다.
`valid=false`이면 신뢰도 부족, 클래스 간 점수 차이 부족 또는 입력 오류이므로
노면 이름이 표시되더라도 확정 판정으로 보지 않는다. 이 모델의 젖음 분류는
눈·얼음도 포함한다. 분류 표시는 `surface_control_enabled=false` 상태에서도 동작한다.

`scripts/run.sh`는 이제 카메라·노면 인식을 기본 실행한다. 기존 서비스 환경에
`SAFESTRIDE_ENABLE_PERCEPTION=false`가 남아 있으면 `true`로 바꿔야 한다.
최초 실행 전 `bash scripts/install_perception.sh`로 의존성을 준비한다.
영상은 추론 노드에서 발행하므로 모델 파일과 OpenCV/NumPy/Torch가 필요하다.

USB 카메라는 `/dev/v4l/by-id/*-video-index0`를 우선 선택한다. 여러 카메라가
연결되면 `SAFESTRIDE_PERCEPTION_CAMERA_DEVICE`에 원하는 장치 경로를 지정한다.
명시한 `SAFESTRIDE_PERCEPTION_CAMERA_INDEX`도 사용할 수 있으며, 둘 다 지정하면
DEVICE가 우선한다. 장치 경로가 없는 환경에서는 인덱스 0으로 폴백한다.

```bash
ls -l /dev/v4l/by-id/
ros2 topic hz /camera/image/compressed
ros2 topic echo /perception/surface_condition --once --qos-reliability best_effort
```

카메라 영상 자체가 없으면 Diagnostics의 `SafeStride/Surface Perception`에서
`camera_open_failed`, `camera_read_failed`, 모델 오류를 확인한다.
수정한 레이아웃 JSON은 Foxglove에서 다시 Import해야 한다.

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

현재 배포는 `terrain_stop_enabled=false`, `range_control_enabled=false`,
`require_range_sensors=false`, `surface_control_enabled=false`,
`require_surface_condition=false`로 ToF·노면 입력을 모니터링에만 사용한다.
MPU 경사 제어와 손잡이·통신 정지는 별개로 유지한다.

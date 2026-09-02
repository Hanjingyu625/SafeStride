# SafeStride

Raspberry Pi 4, Ubuntu Server 24.04, ROS 2 Jazzy와 Arduino Uno 2대로 구성한
스마트 보행기 제어 워크스페이스다.

> 인증된 안전 제어기가 아니다. E-stop이 설치되지 않았으므로 최초 시험은 바퀴를
> 들고 별도의 물리 모터 전원 차단 수단을 준비한 상태에서 진행한다.

## 현재 구성

```text
12 V battery ---------------------------> motor driver
      +-> XL4015 5 V -> Raspberry Pi -> Drive Uno (USB)
                              +--------> Terrain Uno (USB)
                              +--------> BE-220 GPS (serial)

Drive Uno:   shared motor output, left A3 WSH135 Hall, left A2/right A1 pressure
Terrain Uno: downward TOF-10120, GY-521 MPU6050
Raspberry Pi: BE-220 GPS + serial bridges -> safety supervisor -> diagnostics/Foxglove
```

- 왼쪽 휠 WSH135 아날로그 홀센서만 사용하며 A3, 자석 6개로 설정되어 있다. 공통
  드라이브 구조라 오른쪽 ROS 값은 왼쪽 측정값을 복제한 추정치다.
- 압력센서 임계값은 좌우 ADC 80이고 dead-man으로 동작한다.
  별도의 `/walker/set_enabled true` 없이도 정상 링크, fresh `/cmd_vel_safe`,
  Hall/TOF와 양손 압력이 모두 유효하면 자동 arm된다. 정상 압력 해제는 0.6초
  목표속도 ramp 후 정지하고 fault, E-stop, watchdog은 즉시 정지한다.
- TOF는 약 25 cm 아래 지면을 향한다. 초기 기준면 학습 후 EMA 거리,
  적응 기준값, 변화량과 4회 연속 검출을 함께 사용해 높아진 물체와
  낮아진 바닥을 구분한다. 확정 시 모터 명령을 즉시 0으로 만들고 MCU
  watchdog이 재활성화 전까지 정지 상태를 유지한다.
- MPU6050은 3축 가속도·자이로와 중력 기반 roll/pitch를 발행한다. 지자기센서가
  없으므로 yaw는 관측하지 않는다. 5도 이상 pitch가 0.5초 지속되면 경사로
  확정해 내리막은 감속하고 오르막은 목표속도를 높인다. 장착 부호는
  `uphill_pitch_sign`으로 반전할 수 있다. MPU 오류는 경사 보정만 중립화하며
  TOF 단차 안전 정지를 대신하지 않는다.
- GPS는 Raspberry Pi의 별도 serial 장치에서 `gps_node`가 직접 수신한다.
  지도·API가 없으면 횡단보도 노드는
  종료되지 않고 준비 여부만 `/diagnostics`에 표시하며 모터 명령을 발행하지 않는다.
- 배터리 정격은 12 V다. 배터리-ADC 분압 회로가 없으므로 `/battery_state`의
  실제 전압·잔량은 미측정으로 유지한다.

## 빠른 시작

```bash
bash scripts/install_ubuntu_24_04.sh
bash scripts/build.sh
bash scripts/test.sh
SAFESTRIDE_ENABLE_CRUISE=false bash scripts/run.sh
```

운영 직렬 장치는 `/dev/safestride-drive`, `/dev/safestride-terrain`, GPIO UART
`/dev/serial0` 또는 `/dev/ttyS0`이다. `scripts/run.sh`가 GPS UART를 자동으로
선택하며 `SAFESTRIDE_GPS_PORT`로 덮어쓸 수 있다. 펌웨어는
프로토콜 v4이므로 두 Uno와 Pi 소프트웨어를 함께 갱신한다.

```bash
arduino-cli compile --fqbn arduino:avr:uno firmware/safestride_mcu
arduino-cli compile --fqbn arduino:avr:uno firmware/terrain_mcu
```

## 주요 ROS 인터페이스

| 이름 | 형식 | 역할 |
|---|---|---|
| `/wheel/hall` | `WheelHall` | 왼쪽 홀센서 및 미러된 공통 속도 |
| `/handle/pressure` | `HandlePressure` | 좌우 압력과 dead-man 판정 |
| `/terrain/tof` | `sensor_msgs/Range` | TOF 원거리 |
| `/terrain/status` | `TerrainStatus` | 필터·기준·오차·변화량·raised/drop 상태 |
| `/terrain/imu` | `sensor_msgs/Imu` | MPU6050 가속도·자이로·roll/pitch |
| `/gps/fix`, `/gps/speed`, `/gps/course` | GPS 토픽 | 위치·검증된 fallback 속도·이동 중 방향 |
| `/gps/speed_raw` | `std_msgs/Float32` | BE-220 RMC 원시 속도(진단 전용) |
| `/crosswalk/status` | `CrosswalkStatus` | 지도/API/GPS 기반 모니터 결과 |
| `/walker/status` | `WalkerStatus` | Drive MCU 링크·arm·fault 상태 |
| `/diagnostics` | `DiagnosticArray` | 시스템 준비 상태와 오류 원인 |
| `/walker/set_enabled` | `std_srvs/SetBool` | 선택적 수동 inhibit 설정/해제 |

## 검사

- 자동 테스트: `bash scripts/test.sh`
- Pi 연결·실행·topic 확인: [Raspberry Pi 사용 가이드](docs/RASPBERRY_PI_RUN_GUIDE_KO.md)
- 실제 ROS launch integration: `test_safety_supervisor_launch.py`
- Pi와 두 Uno HIL: `bash scripts/hil_smoke_test.sh` (모터를 arm하지 않음)
- 배선·실물 확인: [ROS 하드웨어 테스트](docs/ROS_HARDWARE_TEST_KO.md)
- PC-Pi 끊김 진단: [Pi 연결 진단](docs/PI_CONNECTION_DIAGNOSIS.md)
- 시각화: [Foxglove 구성](docs/FOXGLOVE.md)

휠 반지름, PID, stall/overspeed 제한은 실물 로그를 확보한 뒤 조정한다.

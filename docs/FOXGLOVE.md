# Foxglove 초안 구성

## Pi 설치와 접속

```bash
sudo apt update
sudo apt install ros-${ROS_DISTRO}-foxglove-bridge
```

`/opt/safestride/deploy/systemd/safestride.env`에서 다음 값을 켠 뒤 서비스를
재시작한다.

```text
SAFESTRIDE_ENABLE_FOXGLOVE=true
```

Ubuntu PC와 Pi가 같은 네트워크에 있을 때 Foxglove에서 **Foxglove WebSocket**을
선택하고 `ws://PI_IP:8765`로 접속한다. 설정은 읽기 전용으로 구성되어 Foxglove가
모터 명령을 publish하거나 enable 서비스를 호출할 수 없다.

## 권장 레이아웃

| 패널 | 토픽/필드 | 목적 |
|---|---|---|
| Diagnostics | `/diagnostics` | MCU link, GPS·지도·API 준비 상태 |
| Plot | `/terrain/status.tof_filtered_m`, `tof_reference_m` | 기준면과 필터 거리 비교 |
| Plot | `/terrain/status.tof_error_m`, `tof_change_m` | 단차 판정 튜닝 |
| Raw Messages | `/terrain/status` | raised/drop 후보와 확정 상태 |
| Plot | `/handle/pressure.left_raw`, `right_raw` | 임계값 80 확인 |
| Plot | `/wheel/hall.left_velocity_rad_s` | 왼쪽 D2 홀센서 속도 |
| Map | `/gps/fix` | GPS fix 확인 |
| 3D | `/tf`, `/terrain/imu`, robot model | MPU6050 roll/pitch와 자세 확인 |

첫 실차 시험에서는 Diagnostics, TOF Plot, pressure Plot, wheel Hall 네 패널만
띄워 로그를 남긴다. 이후 `ros2 bag record -s mcap`으로 기록하면 같은 레이아웃을
오프라인 재생에도 사용할 수 있다.

공식 안내: <https://docs.foxglove.dev/docs/getting-started/frameworks/ros2>

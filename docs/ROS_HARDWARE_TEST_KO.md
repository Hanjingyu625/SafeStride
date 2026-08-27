# Raspberry Pi + Arduino 2대 통합 시험

## 1. 전원 없이 빌드

```bash
bash scripts/build.sh
bash scripts/test.sh
arduino-cli compile --fqbn arduino:avr:uno firmware/safestride_mcu
arduino-cli compile --fqbn arduino:avr:uno firmware/terrain_mcu
```

두 Uno를 모두 protocol v4로 다시 업로드한다. ROS bridge와 Arduino 시리얼
모니터는 같은 포트를 동시에 열 수 없다.

## 2. 센서 단독 확인

- 왼쪽 휠 1회전에서 D2 Hall pulse가 6 증가한다.
- A1/A2를 누르면 각각 raw가 25 이상이고 양손에서 dead-man이 true다.
- TOF 정지 기준면은 약 0.25 m이며 초기 10샘플 후 valid가 true가 된다.
- GY-521을 기울이면 `/terrain/imu`와 status roll/pitch가 변한다. MPU가 아직
  연결되지 않은 경우 진단은 WARN이지만 TOF 단차 정지 시험은 계속할 수 있다.
- GPS fix가 없어도 `/gps/fix`는 NO_FIX로 계속 발행된다.

```bash
ros2 topic echo /wheel/hall --once
ros2 topic echo /handle/pressure --once
ros2 topic echo /terrain/status --once
ros2 topic echo /terrain/imu --once
ros2 topic echo /gps/fix --once
ros2 topic echo /diagnostics --once
```

## 3. HIL smoke

모터 전원을 분리하거나 바퀴를 든 상태에서 실행한다. 이 스크립트는 모터를
활성화하지 않는다.

```bash
bash scripts/hil_smoke_test.sh
```

`/terrain/status`의 alert가 다음처럼 변하는지 수동 확인한다.

- 정상 기준면: `TOF_NORMAL`
- TOF에 물체를 가까이 유지: `TOF_CANDIDATE_RAISED` 후 `TOF_RAISED`
- 바닥을 멀리 이동: `TOF_CANDIDATE_DROP` 후 `TOF_DROP`

확정 상태에서 `/cmd_vel_safe`는 0이 된 뒤 송신이 억제되고 Drive Uno가
disarmed 상태로 전환되어야 한다. 장애물을 제거해도 자동 재시작하지 않는다.

## 4. 최종 체크

- `MAGNET_BENCH_MODE=false`, `ENABLE_ESTOP=false`
- `HALL_CALIBRATED=true`, `PRESSURE_THRESHOLDS_CALIBRATED=true`
- 두 ROS YAML에서 `require_range_sensors=true`, `require_deadman=true`
- protocol v4/schema `0x0401`/release `20260826`
- 지도/API가 없을 때 crosswalk 진단은 WARN이고 `/cmd_vel` 발행자는 아니다.
- 시험 종료 후 `/walker/set_enabled false`와 물리 모터 전원 차단 완료

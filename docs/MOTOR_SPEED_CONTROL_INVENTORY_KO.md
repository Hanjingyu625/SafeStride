# 모터 속도 제어 함수·변수·센서 목록

분석 기반: 현재 저장소의 실행 코드·설정이며 5° 확정 내리막 PWM 0 동작을 포함한다.
실기기에서 적용한 ROS 파라미터, 업로드한 MCU 펌웨어, 실제 배선까지 확인한 결과는 아니다.
`scripts/run.sh` 기본 실행 경로의 속도 생성, 제한, 전달, 피드백, 출력, 정지 및 선택 기능을 설명한다.
`prototypes/`의 복사본, 테스트의 가상 센서와 모터는 현재 구동 경로에 포함하지 않는다.
반복문 인덱스, 메시지 발행 객체 등은 속도 알고리즘과 관계있는 역할별로 묶는다.

## 1. 전체 구조와 단위

```mermaid
flowchart TD
  C[CruiseCommandNode: 기본 목표 1.0 m/s] --> S[SafetySupervisor: 속도 제한 및 모드 결정]
  M[MPU6050: pitch] --> T[Terrain Uno / TerrainBridgeNode]
  F[TOF10120: 지면 거리] --> T
  T --> S
  P[양손 FSR 압력] --> U[Drive Uno: 허용 조건 및 제어]
  U --> W[/walker/status]
  W --> S
  S --> D[/drive/command: DriveCommand]
  S --> V[/cmd_vel_safe: 관측용 안전 속도]
  D --> B[SerialBridgeNode: v/r 및 바이너리 명령]
  B --> U
  H[왼쪽 WSH135 Hall / 자석 12개] --> U
  U --> O[공통 PWM + 방향 핀]
  O --> E[드라이버 / 두 모터]
  E --> H
```

- 두 모터는 **하나의 공통 PWM/방향 출력**으로 구동한다. 좌우 독립 PID 제어는 없다.
- 실제 Hall 센서는 왼쪽 하나다. `readHallSamples()`가 오른쪽 데이터를 왼쪽 값으로 복제한다.
- ROS 목표 선속도: `m/s`. MCU 목표/측정 각속도: `mrad/s`. P 제어 계산: `rad/s`.
- PWM 값은 Arduino `0~255` 척도의 count다. 현재 상한 `140`은 약 54.9% 듀티다.
- `v = rω`, `ω_mrad/s = 1000v/r`, 현재 기본 반지름 `r = 0.115 m`.
- 브리지는 실제로 `/drive/command`를 구독한다. `topics.cmd_vel: /cmd_vel_safe` 설정만 보고 TwistStamped가 직접 구동 명령이라고 해석하면 안 된다.
- 이 문서에서 FF는 목표속도/경사로 예상하는 PWM, 피드백은 측정속도와 목표속도의 차이로 수정하는 PWM을 뜻한다.

## 2. 센서와 하드웨어

| 요소 | 연결/입력 | 속도 제어에서의 역할 | 현재 상태 |
|---|---|---|---|
| WSH135 아날로그 Hall | Drive Uno `A3`, 바퀴 자석 12개 | 펄스 간격에서 바퀴 속도 계산, P 피드백, 스톨/과속 검사, ARM 대기 조건 | 사용 |
| 왼손 FSR 압력 | Drive Uno `A2` | 왼손 접촉 여부 | 사용 |
| 오른손 FSR 압력 | Drive Uno `A1` | 오른손 접촉 여부. 양손 모두 있어야 정상 주행 | 사용 |
| GY-521 MPU6050 가속도계 | Terrain Uno I2C, 주소 `0x68/0x69` | 전후 경사 `pitch` 계산 → 오르막 PWM FF, 확정 내리막 PWM 0, 무효 IMU BRAKE | 사용 |
| MPU6050 자이로 | 같은 보드 | 각속도 telemetry. 현재 pitch 계산에 자이로 적분/융합을 사용하지 않음 | 관측 |
| TOF10120 하향 거리 센서 | Terrain Uno I2C `0x52` | 기준 지면보다 가까운 돌출물/먼 낙차 확정 → 3초 PWM 감속 | 사용 |
| 카메라/노면 분류 모델 | Pi V4L2 카메라 | `recommended_speed_scale` 생성 가능 | 인식 노드는 실행 가능하지만 `surface_control_enabled=false`로 속도 반영 꺼짐 |
| BE-220 GPS | Pi `/dev/serial0` | 위치/속도/방향 → 횡단보도 상위 정책. MCU 속도 P 루프의 입력은 아님 | 횡단보도 감시용 |
| 전방 좌우 거리 센서 | `/range/front_left`, `/range/front_right` | 유효 데이터가 오면 장애물 속도 제한 | Drive 센서 미구현, 무효 sentinel 송신 |
| E-stop 입력 | Drive `12` 예정 | 즉시 정지 | `ENABLE_ESTOP=false` |
| 드라이버 fault 입력 | Drive `13` 예정 | fault latch 및 즉시 정지 | `USE_DRIVER_FAULT_PIN=false` |
| 배터리 ADC | Drive `A5` 예정 | 배터리 telemetry | `ENABLE_BATTERY_SENSE=false`; 저전압 속도 보상 없음 |
| 좌우 전류 ADC | Drive `A0/A4` 예정 | 전류 telemetry | `ENABLE_CURRENT_SENSE=false`; 전류 제한 제어 없음 |
| 모터 드라이버 | PWM `5`, IN1 `6`, IN2 `8` | 공통 PWM과 회전 방향을 실제 출력으로 변환 | 사용 |

압력을 세게 누른 만큼 빨라지는 제어는 없다. FSR은 주행 허용과 해제 감속에 관여한다.
Hall은 회전 방향을 측정하지 않는다. 속도 부호/펄스 위치 부호는 명령 방향에서 추정한다.

## 3. 목표속도 생성: CruiseCommandNode

소스: `src/safestride_control/safestride_control/cruise_command_node.py`.

| 함수/변수 | 설명 |
|---|---|
| `CruiseCommandNode.__init__()` | 기본 목표와 발행 주기를 검증하고 타이머/발행기를 생성 |
| `_publish_command()` | `/cmd_vel`에 `twist.linear.x = _speed`, `angular.z = 0` 발행 |
| `_speed` / `speed_mps` | 기본 `1.0 m/s`; 허용 파라미터 범위 `0~1.0` |
| `publish_rate_hz` / `_timer` | 기본 20 Hz로 새로운 목표를 계속 공급 |
| `command_topic`, `_frame_id`, `_publisher` | 명령 전달 대상, 좌표계, 발행 객체 |
| `main()` | ROS 노드 실행/종료 |

이는 기본 요청값이다. 실제 바퀴 속도는 뒤의 모든 제한과 피드백을 거친다.

## 4. Pi 안전 감독: SafetySupervisor

소스: `src/safestride_control/safestride_control/safety_supervisor_node.py`.

### 함수

| 함수 | 역할 |
|---|---|
| `__init__()` | 정책·상태 변수·구독·발행·주기 구성, 파라미터 검증 |
| `_command_callback()` | 원래 목표와 수신시각 저장 |
| `_status_callback()` | Drive 상태/손 접촉/고장/통신 상태 저장 |
| `_terrain_callback()` | ToF/MPU 상태와 수신시각 저장 |
| `_surface_callback()` | 노면 분류/권장 배율 저장 |
| `_range_callback()` | 전방 거리 범위/유효성 판정 및 수신시각 저장; 정상 범위가 있으면 `+inf`를 최대거리로 해석 |
| `_now_seconds()`, `_age()` | 시각/데이터 나이 계산; 없거나 시각이 역행하면 무효 나이 |
| `_status_reasons()` | link, state, estop, watchdog, fault, armed, deadman, telemetry age 검사 |
| `_command_reasons()` | 목표 누락/0.5초 초과/비유한값/지원하지 않는 회전 명령 검사 |
| `_terrain_reasons()` | 현재 `terrain_stop_enabled=false`로 감시만 수행. 활성화하면 확정 위험 시 정지 상태 진입; 유효 정상 0.5초 및 시작 후 3.1초 조건으로 해제 |
| `_slope_state()` | MPU 유효성·나이 검사 후 경사 정책에 전달 |
| `_range_scale()` | 거리 ≤0.35m: 0, ≥0.80m: 1, 사이: 선형 배율 |
| `_range_state()` | 유효하고 최근인 좌우 전방 거리에서 배율 생성. 없는 센서는 중립 배율 |
| `_surface_state()` | 기능이 켜졌을 때 노면 상태·confidence·배율·나이 검증 |
| `_desired_command()` | 전후진/회전 상한 → 거리 배율 → 노면/경사 배율 → 다시 상한 적용. 후진에 오르막 보조를 적용하지 않음 |
| `_slew()` | `현재 + clamp(목표-현재, ±변화율×dt)`로 선속도/회전속도 변화를 제한 |
| `_timer_callback()` | 모든 조건 종합, 안전 속도와 `DriveCommand` 발행. 위험은 속도 0 및 BRAKE/감속 모드 |
| `_publish_diagnostics()`, `_bool_text()` | 판정 이유와 제한 상태 표시; PWM 계산에는 직접 사용하지 않음 |
| `_clamp()` | 수치 상하한 제한 |
| `main()` | ROS 노드 실행/종료 |

### 속도·정책 파라미터와 저장 변수

| 파라미터 → 변수 | 현재 기본값/역할 |
|---|---|
| `max_forward_velocity` → `_max_forward` | `1.15 m/s` 전진 상한 |
| `max_reverse_velocity` → `_max_reverse` | `0.08 m/s` 후진 크기 상한 |
| `max_angular_velocity` → `_max_angular` | 실행 YAML `0.0 rad/s`; 회전 제어 제한 |
| `max_linear_acceleration` → `_linear_accel` | `0.20 m/s²` 목표 선속도 상승 제한 |
| `max_linear_deceleration` → `_linear_decel` | `0.50 m/s²` 정상 목표 선속도 하강 제한 |
| `max_angular_acceleration`, `max_angular_deceleration` → `_angular_accel`, `_angular_decel` | `0.50/1.00 rad/s²`; 현재 회전 목표는 0 |
| `drive_pwm_cap` → `_drive_pwm_cap` | `140 count` PWM 상한 |
| `slope_control_enabled` → `_slope_control_enabled` | `true` |
| `uphill_pitch_sign`, `pitch_offset_rad` | `-1.0`, `0.0`; Terrain bridge 보정 후 `normalized_pitch=(pitch-offset)×sign` |
| `slope_enter_angle_rad`, `slope_exit_angle_rad` | `5°`, `3°`; 경사 분류 히스테리시스 |
| `slope_confirmation_time_s` | `0.50 s`; 후보 경사가 지속되어야 분류 확정 |
| `uphill_speed_scale`, `downhill_speed_scale` | `1.0`, `1.0`; 목표 배율은 유지. 확정 내리막은 별도 BRAKE/PWM 0 처리 |
| `max_combined_speed_scale` → `_max_combined_speed_scale` | `1.25`; 노면/경사 증속 배율 상한 |
| `brake_enter_deg`, `brake_release_deg`, `brake_recovery_s` | 내리막 `7°` 진입, `4°` 이하로 완화되어 `0.5 s` 유지 시 해제 |
| `terrain_stop_enabled` | `false`; 확정 ToF 위험도 현재 모터 제어에 미반영 |
| `require_deadman` → `_require_deadman` | `true`; 양손 해제는 감독 단계에서 정지 이유 |
| `surface_control_enabled` → `_surface_control_enabled` | `false`; 현재 노면 배율 무시 |
| `require_surface_condition` → `_require_surface` | `false`; 기능 활성 시 노면 누락/무효를 필수 정지 조건으로 삼을지 |
| `max_surface_speed_scale` → `_max_surface_scale` | 코드 기본 `1.25`; 유효 노면 배율 검증 상한 |
| `require_range_sensors` → `_require_ranges` | `false`; 현재 코드에서는 이 값으로 ToF 누락 정지를 시행하지 않음 |
| `stop_distance`, `slow_distance` → `_stop_distance`, `_slow_distance` | 전방 거리 `0.35/0.80 m` |
| `publish_rate` → `_publish_rate` | `20 Hz` 감독 계산/명령 발행 |
| `command_timeout`, `status_timeout`, `max_telemetry_age` → 대응 `_...` | 각각 `0.50 s`; 오래된 목표/상태 차단 |
| `range_timeout` → `_range_timeout` | `0.35 s`; 전방/ToF/MPU 최근성 검사 |
| `surface_timeout` → `_surface_timeout` | 코드 기본 `2.5 s` |
| `_last_command`, `_last_status`, `_last_terrain`, `_last_surface` 및 각각 `_..._time` | 최신 입력과 수신시각 |
| `_ranges` | 좌우 `distance`, `valid`, `time` 저장 |
| `_output_linear`, `_output_angular`, `_last_tick_time` | 현재 제한된 목표 및 slew 계산 시각 |
| `_slope_policy`, `_brake_policy` | 경사 분류와 정지/해제 상태 객체 |
| `_slope_braking`, `_slope_ff_pwm` | 7° 조기 감속 여부와 추가 FF PWM |
| `_terrain_stopping`, `_terrain_stop_started`, `_terrain_clear_since`, `_terrain_note` | ToF 감속 상태, 최소 유지/정상 복귀 시간, 진단 이유 |
| `_command_output_suppressed` | 위험 때 한 번 정지 명령 후 발행 중단; 브리지 timeout에 의한 disable을 유도 |
| `requested_linear/angular`, `desired_linear/angular`, `range_scales`, `surface_scale`, `slope_scale`, `combined_speed_scale`, `dt` | 매 주기 입력 목표, 계산 목표, 배율 및 실제 시간차 |
| `hard_stop_reasons`, `motion_stop_reasons`, `operating_notes`, `may_stream_command` | 즉시 차단·동작 제한·발행 지속 여부. 단순 `disarmed`만 있으면 새 목표를 계속 보내 자동 ARM 가능 |
| `diagnostic_rate`, `_diagnostic_rate`, `_last_diagnostic_time`, `_last_log_summary` | 진단/로그 주기 및 중복 표시 억제 |
| `command_topic`, `safe_command_topic`, `drive_command_topic`, 각 센서/status topic, `_output_frame_id` | 데이터 연결/좌표계; 발행·구독 객체가 실제 전달 담당 |

### 경사 계산 함수와 상태

소스: `src/safestride_control/safestride_control/safety_logic.py`.

| 함수/변수 | 설명 |
|---|---|
| `finite_parameter()` | NaN/inf, bool, 범위 밖의 제어 설정을 거부 |
| `combine_speed_scales()` | 어느 한 배율이 1 미만이면 최소값 선택. 둘 다 1 이상이면 곱한 뒤 상한 적용 |
| `SlopeSpeedPolicy.__init__()` | 경사 진입/해제·확정시간·부호·offset·배율 저장 |
| `reset()` | LEVEL 및 후보/확정시간 초기화 |
| `_desired_state()` | 현재 상태와 정규화 pitch로 UPHILL/LEVEL/DOWNHILL 후보 결정 |
| `update()` | 유효성, 히스테리시스, 후보 지속시간 검사 후 배율/상태/pitch 반환 |
| `_enter_angle`, `_exit_angle`, `_confirmation_time` | 5°/3°/0.5초 분류 기준 |
| `_uphill_pitch_sign`, `_pitch_offset`, `_downhill_scale`, `_uphill_scale` | 보정 부호/offset/배율 |
| `_state`, `_candidate`, `_candidate_since` | 확정 상태, 후보 상태, 후보 시작시각 |
| `SlopeBrakePolicy.__init__()`, `update()` | 내리막 정지 히스테리시스와 정상 복귀 시간 검사. 비유한 pitch도 정지 |
| `enter`, `release`, `recovery`, `braking`, `clear_since` | 7°/4°/0.5초와 현재 정지/복귀 상태 |
| `slope_feedforward_pwm()` | 오르막에서 `round(clamp(5×(각도_deg-3), 0, 30))`; 내리막은 0 |

Terrain bridge가 평지 기준 pitch -0.116rad, roll -0.059rad를 `/terrain/status`에서 보정한다. 감독 정책은 보정 후 pitch의 부호를 뒤집는다.
오르막 5°가 확정되면 기본 요청 1.0m/s를 유지하고 경사 FF 10count를 추가한다. 실제 속도 상승은 보장되지 않는다.
5° 이상 내리막이 0.5초 지속되어 확정되면 목표 0의 `BRAKE`로 PWM을 즉시 0으로 만든다.
확정 전 7° 이상은 기존 `TERRAIN_STOP` 3초 감속을 먼저 시작하며, IMU 누락/무효/오래됨은 `BRAKE`로 보낸다.
YAML의 'invalid MPU neutral scale' 주석보다 실제 `_timer_callback()` 및 BRAKE 정책 동작을 우선한다.

## 5. Pi ↔ Drive 명령 전달: SerialBridgeNode

소스: `src/safestride_bridge/safestride_bridge/serial_bridge_node.py`.

| 함수 | 역할 |
|---|---|
| `__init__()`, `_declare_parameters()`, `_load_parameters()`, `_value()` | 통신/시간/기구 파라미터와 상태 초기화·검증 |
| `_on_cmd_vel()` | 이름과 달리 `DriveCommand` 수신. 선속도, header 나이, FF, PWM cap, mode 및 조합 검증 |
| `_on_set_enabled()` | `/walker/set_enabled`: false면 enable 차단 및 disable 전송, true면 물리조건/새 명령에 따른 enable 허용 |
| `_command_tick()` | 50Hz로 link/원격조건/명령 최근성 확인 → `ω=v/r` → 상한 → COMMAND 전송 |
| `_send_command()` | 각속도·TTL·enable·FF·cap·mode를 `CommandPayload`로 직렬화 |
| `_serial_connected()`, `_link_ok()` | 포트/세션/최근 telemetry 조건 검사 |
| `_firmware_state()`, `_firmware_status_consistent()` | state bit 추출 및 모터 enabled/bench 상태 일관성 검사 |
| `_remote_allows_enable()` | 양손, Hall capability/보정 옵션, watchdog, estop, fault, state로 enable 허용 판단 |
| `_remote_allows_deadman_ramp()` | 손 해제 중 ARMED이면 목표 0/enable 유지 명령만 전송하여 MCU 자체 600ms ramp 지원 |
| `_magnet_bench_mode_active()` | 보드 capability/status 및 Pi의 bench 허용 설정 확인 |
| `_try_open_serial()`, `_close_serial()`, `_reset_link_state()` | 포트 재연결/연결 해제/세션 및 최근 입력 초기화 |
| `_io_tick()`, `_handle_frame()`, `_handle_hello()` | 수신, CRC/세션/순번/규약 검사, 보드 호환성 확인 및 세션 시작 |
| `_next_sequence()`, `_make_frame()`, `_write_frame()` | 새 순번/프레임 생성과 송신. 실패하면 통신 중단 처리 |
| `_now_monotonic()`, `_stamp()` | timeout용 단조시각 및 ROS header 시각 |
| `_publish_telemetry()` | 펄스/각속도를 Hall, joint state, odometry, 범위/압력/배터리 출력으로 변환 |
| `_int32_delta()`, `_quaternion_z()` | 누적 펄스의 정수 wrap 차분 및 odometry 자세 표현 |
| `finite_float()`, `bounded_int()` | 외부 파라미터의 유한값/정수 범위 검증 helper |
| `_publish_status()`, `_walker_constant()` | 측정속도/PWM/유효성/주행상태를 `/walker/status`로 전달하여 감독 조건으로 되돌림 |
| `_publish_pressure()`, `_publish_range()`, `_publish_battery()` | 개별 센서 telemetry 발행; 압력 크기로 목표 생성하지 않음 |
| `_diagnostic_tick()` | 통신 및 제어값 관측 |
| `destroy_node()`, `main()` | 종료 시 정지 명령 및 노드 수명 관리 |

| 변수/파라미터 | 설명/현재 값 |
|---|---|
| `_target_linear`, `_slope_ff_pwm`, `_drive_pwm_cap`, `_drive_mode` | 최신 원자적 목표/FF/cap/mode |
| `_last_command_time`, `_command_timed_out`, `_command_timeout` | 목표 최근성 `0.50 s` |
| `_level_enable_blocked` | 서비스로 설정한 지속 enable 차단 |
| `_command_rate_hz`, `_command_ttl_ms` | `50 Hz`, `200 ms`; MCU는 실제 수신 TTL 감시 |
| `_wheel_radius`, `_wheel_separation` | 기본 `0.115/0.55 m`; 반지름은 목표 환산, 간격은 odometry 회전 계산 |
| `_max_wheel_speed` | `10.0 rad/s` 상한, 즉 기본 반지름에서 `1.15 m/s` |
| `_hall_pulses_per_revolution` | `12`; 펄스 위치를 회전각으로 환산 |
| `_deadman_direct_drive`, `_deadman_forward_velocity` | `false`, `0.10 m/s`; 켜지면 정상 supervised 목표 대신 고정 목표를 사용하는 선택 경로 |
| `_require_hall_calibration` | `false`; 인증 플래그로 enable을 막지는 않음 |
| `_allow_magnet_bench_mode`, `_auto_arm_magnet_bench_mode` | 둘 다 `false` |
| `_max_abs_angular_z` | 현재 `0`; 현재 DriveCommand/공통 모터 구조에는 좌우 회전 목표 필드가 없음 |
| `_serial`, `_lock`, `_parser`, `_port`, `_baudrate`, `_read_chunk_size`, `_max_frame_size`, `_poll_rate_hz`, `_reconnect_period`, `_last_open_attempt` | 직렬 연결·동기화·프레임 해석 상태; 기본 115200baud/100Hz poll/1초 reconnect |
| `_session_id`, `_boot_id`, `_capabilities`, `_session_started`, `_tx_sequence`, `_compatibility_error` | 올바른 보드/부팅/세션/규약인지 확인하는 상태 |
| `_last_telemetry`, `_last_telemetry_time`, `_last_telemetry_sequence`, `_telemetry_timeout` | 원격 상태·나이·순번; link 최근성 `0.30 s` |
| `_payload_error_count`, `_session_error_count`, `_sequence_error_count` 및 parser error counters | 거부된 프레임/통신 오류 진단 |
| `_last_hall_left/right`, `_joint_left/right`, `_odom_x/y/yaw` | 펄스 차분과 위치 적분. MCU PWM P 루프에 위치를 되먹이지는 않음 |
| `target_linear`, `target`, `received_at`, `fresh`, `link_ok`, `remote_safe`, `enable` | 명령 송신 단계의 환산/최근성/허용 판정 지역값 |
| range/battery/frame/joint/topic 설정 및 발행·구독·타이머 객체 | 센서 출력 형식과 연결; 정상 FF/P 게인은 바꾸지 않음 |

브리지와 MCU의 반지름 모델을 같이 확인해야 한다. Pi 반지름은 launch에서 덮어쓸 수 있지만 MCU의 nominal/과속 상수에는 `0.115`가 직접 들어 있다.

## 6. Drive Uno 스케줄·허용 조건·명령 수신

소스: `firmware/safestride_mcu/safestride_mcu.ino`, `controller_state.h`.

### 함수

| 함수 | 역할 |
|---|---|
| `setup()` | 모터 출력부터 0으로 초기화, 센서/통신/부팅 ID 설정, AVR 500ms 하드웨어 watchdog 활성 |
| `loop()` | 물리조건 → 압력 갱신 → watchdog → 직렬 명령 → watchdog → 제어 → telemetry |
| `runControlLoop()` | 최소 5ms마다 Hall 갱신/스냅샷 → 출력 허용 → DriveController 호출 → 고장/손 해제 완료/ARM 대기 판단 |
| `readHallSamples()` | 왼쪽 count/period/age 취득, 오른쪽에 복제 |
| `deadmanActive()` | `bothHandsPresent()` 결과 |
| `motionDeadmanSatisfied()` | bench 또는 deadman 비필수 설정 또는 양손 접촉 |
| `estopActive()`, `driverFaultActive()` | 기능이 켜졌을 때 디지털 정지/fault 입력 판정 |
| `refreshPhysicalSafety()` | estop/fault 즉시 정지, 손 해제 감속 진입, 재접촉하면 손 해제 감속 취소 |
| `beginDeadmanReleaseRamp()` | 현재 applied target 크기를 600ms 내 0으로 만드는 감속률 계산, 요청을 0으로 |
| `immediateStop()` | 목표/손 해제 상태 초기화, 제어 상태 변경, `disableImmediately()` 호출 |
| `handleCommand()` | session/새 순번/payload/목표/TTL/enable/FF/cap/mode/물리조건/state 검증 후 적용 |
| `markAcceptedCommand()` | **수락된 새 명령만** 마지막 수신시각/TTL/순번/watchdog 상태 갱신 |
| `stationaryDwellMet()` | ARM 대기용 속도 조건이 250ms 유지되었는지 |
| `enforceWatchdogs()` | ARMED에서 명령 TTL 초과 시 즉시 SAFE_STOP/세션 무효; 세션 활동 1초 초과 시 연결 해제 |
| `invalidateSessionForWatchdog()`, `clearSession()` | watchdog 정지/세션 종료 및 enable 초기화 |
| `processSerial()` | 완전하고 검증된 프레임만 session/command 처리기로 전달 |
| `handleSessionStart()`, `sendHello()`, `makeBootId()` | 보드 역할/규약/EEPROM boot ID로 이전 부팅 명령 배제. 연결만으로 ARM하지 않음 |
| `sendTelemetry()`, `currentStatusBits()` | 측정속도/PWM/손 접촉/고장/수락 순번을 Pi에 송신 |
| `readRangeLeftMm()`, `readRangeRightMm()` | 미구현 전방 센서; `0xFFFF` 반환 |
| `readBatteryMv()`, `readCurrentMa()` | 선택 ADC telemetry; 현재 무효 sentinel 반환 |
| `elapsedMs()` | unsigned 시간차 계산 |

### 전역 상태 변수

| 변수 | 역할 |
|---|---|
| `g_drive`, `g_hall`, `g_pressure`, `g_receiver` | 모터 제어, Hall, 압력, 프레임 수신 객체 |
| `g_requested_mrad_s` | 받은 각속도 목표 |
| `g_slope_ff_pwm`, `g_drive_pwm_cap` | 추가 경사 PWM, 최종 상한 |
| `g_brake_requested`, `g_terrain_stop_requested` | 명시적 즉시 BRAKE, 3초 PWM 감속 모드 |
| `g_state` | BOOT/DISARMED/ARMED/SAFE_STOP/ESTOP/FAULT |
| `g_fault_bits`, `g_watchdog_timed_out` | latched fault와 watchdog 정지 상태 |
| `g_deadman_release_ramp_active`, `g_deadman_release_decel_mrad_s2` | 손 해제 감속 여부/감속률 |
| `g_stationary_tracking`, `g_stationary_since_ms` | ARM 대기 추정 정지 조건의 유지 여부/시각 |
| `g_session_active`, `g_session_offer_active`, `g_boot_id`, `g_session_id` | 연결/세션/부팅 상태 |
| `g_valid_command_seen`, `g_have_command_sequence`, `g_last_command_sequence` | 유효 명령 수신 여부와 순번 |
| `g_active_command_ttl_ms`, `g_last_valid_command_ms`, `g_last_session_activity_ms` | 명령/세션 기한 |
| `g_tx_sequence`, `g_last_control_us`, `g_last_telemetry_ms`, `g_last_hello_ms` | 송신 순번 및 실행 스케줄 |
| `g_new_pulse_since_telemetry` | 송신 사이 새 Hall 펄스 발생을 누적 표시 |
| `output_allowed` | session 활성, ARMED, estop 없음, 양손 또는 해제 ramp, fault 없음의 AND |
| `elapsed_us`, `left_hall/right_hall`, `stationary`, `left_speed/right_speed`, `hall_faults` | 시간차/센서 스냅샷/ARM 추정 조건/고장 지역값 |

`STATUS_*`는 session/enabled/deadman/estop/watchdog/유효 명령/Hall 보정/bench/state bit,
`CAP_*`는 보드의 Hall/범위/배터리/전류/deadman/estop/압력/bench 기능 bit다.
현재 실제 설정하는 고장은 `FAULT_MOTOR_DRIVER`와 `FAULT_LEFT_HALL`이다.
`PRESSURE_FLAG_*`는 양손 접촉과 보정 여부를 운반한다.

## 7. 실제 PWM 계산: DriveController

소스: `firmware/safestride_mcu/motor_control.h`, `motor_control.cpp`.

### 정상 계산식

```text
ω_target = clamp(요청 각속도, -10000, +10000) 후 목표 ramp
ω_measured = 왼쪽 Hall 펄스 주기 속도의 필터값
ω_nominal = (1.0 / 0.115) × 1000 ≈ 8695.65 mrad/s

|ω_target| < 20 이면 기본 FF = 0
그 외 기본 FF = sign(ω_target) × [30 + (60-30)|ω_target|/ω_nominal]
전진일 때 FF += clamp(slope_ff_pwm, -60, +30)
P = speed_valid ? 12 × (ω_target-ω_measured)/1000 : 0
출력 후보 = FF + P
→ 목표 반대 방향 금지
→ min(Pi pwm_cap, MCU MAX_PWM=140) 제한
→ 무효 Hall이면 후보 크기 60 제한
→ 한 번의 전진 시작 step 최대 20
→ 정상 상승 20count/s, 지형 정지 복귀 상승 10count/s, 하강 60count/s
→ 상한/방향 재적용 → writeMotor()
```

Ki=Kd=0이고 이를 `static_assert`로 강제한다. 함수 이름은 PID지만 현재 실제 보정은 P만 동작한다.
FF bias 30은 최저 출력 30을 강제하는 값이 아니다. P 보정/상한/감속으로 최종 PWM은 30 아래 또는 0이 될 수 있다.
Hall 무효 시 60 제한은 출력 후보에 적용된다. 이미 더 높은 출력에서 Hall이 무효가 된 경우 최종 출력은 하강 slew를 거쳐 내려간다.

### 함수

| 함수 | 역할 |
|---|---|
| `DriveController()` | 모든 속도/필터/PID/펄스/고장 상태 초기화 |
| `begin()` | PWM/방향 핀 구성 및 0 출력 |
| `update()` | 정상 제어/정지/손 해제/지형 감속/Hall fault/과속 경로의 중심 |
| `rampTarget()` | 목표 각속도 상승 1200mrad/s², 하강 기본 2500mrad/s² 또는 지정 감속률로 제한 |
| `calculatePid()` | rad/s 오차와 I/D 상태 계산; 현재 결과 `12×오차` |
| `compensateMotorDeadzone()` | 비유한값/작은 목표(<20mrad/s)를 0 처리, 목표 반대 방향 출력 금지. 최소 PWM을 더하지 않음 |
| `openLoopPwm()` | Hall enforcement를 껐을 때 목표 크기에 따라 bias~MAX_PWM 개방루프 출력 |
| `writeMotor()` | PWM 범위/물리부호 적용, PWM을 제거한 뒤 방향 핀 변경, 새 magnitude 출력. 역전에는 150ms BRAKE |
| `hallSpeedMagnitude()` | `(2000π/12)×1000000/period_us`; 첫 펄스만 있거나 너무 오래됐으면 0 |
| `updateHallFeedback()` | 펄스 차분/명령 방향 위치/속도 유효성/새 펄스/EMA 갱신 |
| `updateHallMonitor()` | 출력 중 펄스 없음의 누적시간, 새 펄스에서 비현실적 과속 횟수 검사 |
| `updateHallPlausibility()` | 현재 왼쪽 Hall만 감시하고 `HALL_FAULT_LEFT` latch |
| `disableImmediately()` | PWM=0, IN1=IN2=LOW로 코드상 BRAKE; 목표/PID/PWM 이력 초기화 |
| `clearRecoverableFaults()` | Hall 고장/감시/PID 상태 초기화 |
| `updateMagnetBench()` | 선택 시험 모드: 최근 자석 펄스 창에서 고정 ±60PWM. 정상 속도 제어/압력/스톨 감시 우회 |
| `leftVelocityMradS()`, `rightVelocityMradS()` | 필터 각속도 반환 |
| `appliedTargetMradS()` | ramp된 목표 반환 |
| `leftHallPulsePosition()`, `rightHallPulsePosition()` | 명령 부호를 붙인 누적 펄스 위치 반환 |
| `feedbackReady()` | 제어 피드백 업데이트를 두 번 거쳤는지. 실제 유효 펄스 두 개라는 뜻은 아님 |
| `speedValid()`, `newPulse()`, `speedAgeUs()` | 속도 유효성/새 왼쪽 펄스/마지막 펄스 나이 |
| `feedforwardPwm()`, `feedbackPwm()`, `appliedPwm()`, `braking()`, `hallFaultMask()` | 제어/출력/고장 상태 조회 |
| `clampFloat()`, `roundedInt32()`, `magnitudeInt32()` | 범위 제한, 포화 반올림, 안전한 정수 절댓값 |
| `updateTimer()` | 조건부 누적시간, overflow 포화 |
| `hallZeroTimeoutUs()`, `velocityFilterAlpha()` | 정상/bench에 맞는 속도 유효기간 및 필터 계수 선택 |
| `hallStallTimeoutUs()` | 예상 자석 간격×2.5, 최소 5초/최대 10초의 적응형 스톨 기한 |

### 구조체·멤버·인자

| 이름 | 역할 |
|---|---|
| `HallSample.pulse_count`, `.period_us`, `.age_us` | 누적 펄스, 마지막 두 펄스 시간차, 마지막 펄스 나이 |
| `PidState.integral`, `.previous_error` / `motor_pid_` | I 누적값/D 이전 오차. 현재 Ki/Kd=0으로 PWM 기여 없음 |
| `HallMonitorState.initialized`, `.previous_count`, `.no_pulse_us`, `.overspeed_pulses` | 감시 초기화/새 펄스/무펄스 지속시간/과속 관측 횟수 |
| `left_hall_monitor_`, `right_hall_monitor_`, `hall_fault_mask_`, `HALL_FAULT_LEFT` | 감시 상태 및 왼쪽 고장 bit; 오른쪽 독립 감시는 하지 않음 |
| `filtered_left_mrad_s_`, `filtered_right_mrad_s_` | 필터 속도. 현재 오른쪽은 복제 센서 추정값 |
| `applied_target_mrad_s_` | 가감속 제한 후 적용 목표 |
| `feedback_initialized_`, `feedback_sample_count_` | 펄스 차분 초기화 및 업데이트 횟수 |
| `previous_left_pulse_count_`, `previous_right_pulse_count_` | 전 주기 누적 count |
| `left_position_bits_`, `right_position_bits_`, `feedback_direction_` | 부호 추정 누적 위치/명령 기반 방향 |
| `speed_valid_`, `new_pulse_`, `speed_age_us_` | 왼쪽 속도 유효성/새 측정/나이 |
| `ff_pwm_`, `feedback_pwm_` | 기본+경사 FF 및 P 보정 |
| `last_commanded_pwm_`, `applied_pwm_counts_` | 최종 PWM 이력 및 정수 출력값 |
| `braking_`, `startup_pending_` | 현재 0 출력 여부 및 다음 전진 시작 step 허용 여부 |
| `last_drive_direction_`, `reversal_remaining_us_` | 마지막 구동방향 및 역전 BRAKE 잔여시간 |
| `release_start_pwm_`, `release_pwm_fade_active_` | 손 해제 시작 PWM/감속 상태 |
| `terrain_stop_active_`, `terrain_stop_elapsed_us_`, `terrain_start_pwm_` | 지형/경사 감속 상태·경과시간·시작 PWM |
| `terrain_recovering_` | 감속 복귀 때 느린 상승 slew 유지 |
| `speed_brake_`, `absolute_overspeed_pulses_` | 절대 과속 BRAKE 상태 및 확인 펄스 횟수 |
| `elapsed_us`, `dt_seconds` | 실제 주기 시간; 목표 ramp/PID/PWM slew/정지 타이머에 사용 |
| `requested_mrad_s`, `left_hall`, `right_hall` | 목표와 센서 입력 |
| `output_allowed`, `enforce_hall_faults` | 최종 허용조건 및 정상 Hall 피드백/고장 감시 경로 선택 |
| `deceleration_mrad_s2`, `fade_pwm_during_deceleration` | 감속률 및 손 해제 실제 PWM fade 사용 여부 |
| `slope_ff_pwm`, `pwm_cap`, `brake_requested`, `terrain_stop_requested` | 경사 추가량, 출력 cap, 즉시/3초 감속 모드 |
| `target_rad_s`, `measured_rad_s`, `error`, `derivative` | PID 수식 지역값 |
| `limited_target`, `target`, `direction`, `raw_speed`, `raw_left/right`, `alpha`, `left_delta/right_delta` | 목표/측정/필터/펄스 차분 지역값 |
| `output`, `desired_output`, `cap`, `rise`, `rate`, `terrain_recovery_complete` | 출력 후보/상한/slew/복귀 완료 판정 |
| `ratio`, `terrain_cap`, `starting_magnitude`, `remaining_ratio` | 지형/손 해제 PWM 감속 비율과 단조 감소 상한 |
| `logical_pwm`, `signed_pwm`, `magnitude`, `applied` | 논리 PWM/물리 배선 방향/실제 출력 크기 |

### 정지·감속 경로

| 경로 | 실제 동작 |
|---|---|
| 출력 금지 / fault / watchdog / 명시적 BRAKE | 정상 FF/P/slew를 기다리지 않고 0 PWM |
| 일반 목표 0 | 손 해제 fade/terrain 모드가 아니면 즉시 BRAKE |
| 양손 중 하나 해제 | 해제 확정 후 현재 목표와 직전 실제 PWM을 약 600ms 동안 줄임; 그 뒤 SAFE_STOP |
| 확정 전 7° 내리막 또는 ToF 정지 기능 활성화 시 확정 위험 | 시작 실제 PWM을 3초 동안 선형으로 0으로 감소. 5° 내리막 확정 BRAKE가 들어오면 즉시 0 |
| 5° 내리막 0.5초 확정 | 목표 0과 명시적 BRAKE로 정상 slew를 우회해 PWM 즉시 0, IN1=IN2=LOW |
| 절대 과속 | **필터 전** 왼쪽 Hall 속도가 8km/h 초과인 새 펄스 간격 2번이면 BRAKE. 7km/h 미만 새 측정 또는 5초 timeout에서 해제 |
| Hall 스톨 | 목표 ≥20mrad/s 및 실제 출력 존재 중 새 펄스가 5~10초 없으면 fault |
| 비현실적인 Hall 속도 | **필터 속도** 25000mrad/s 초과를 새 펄스 2번에서 관측하면 fault |
| 방향 역전 | 최소 150ms 0 PWM/BRAKE를 끼운 뒤 새 방향 적용 |

코드상 `braking=true`는 0 PWM 및 00 방향 핀 출력 상태다. 물리적 정지/경사 위치 고정을 측정해 확인했다는 뜻은 아니다.
절대 과속 BRAKE와 Hall plausibility fault는 문턱/측정값/복귀 방식이 서로 다르다.

## 8. Drive 설정 상수 전체

소스: `firmware/safestride_mcu/config.h`.

| 상수 | 현재 값 | 역할 |
|---|---|---|
| `SERIAL_BAUD` | 115200 | 제어 명령/상태 통신 |
| `CONTROL_PERIOD_US`, `TELEMETRY_PERIOD_MS` | 5000 / 20 | 200Hz 제어, 50Hz 상태 송신 |
| `HELLO_PERIOD_MS`, `SESSION_LOSS_TIMEOUT_MS` | 500 / 1000 | 연결/세션 시한 |
| `COMMAND_WATCHDOG_MAX_MS`, `COMMAND_TTL_MIN_MS` | 250 / 20 | 수락 TTL 범위; 현재 Pi는 200ms 전송 |
| `MAX_WHEEL_TARGET_MRAD_S` | 10000 | 각속도 요청 상한 |
| `MAX_ACCEL_MRAD_S2`, `MAX_DECEL_MRAD_S2` | 1200 / 2500 | MCU 목표 ramp 변화율 |
| `ARM_MAX_MEASURED_SPEED_MRAD_S`, `ARM_STATIONARY_DWELL_MS` | 100 / 250 | ARM용 추정 저속 유지 조건 |
| `DEADMAN_DIRECT_DRIVE`, `DEADMAN_RELEASE_RAMP_MS` | false / 600 | 정상 Hall 루프 선택 및 손 해제 감속시간 |
| `HALL_ANALOG_PIN`, `HALL_SAMPLE_PERIOD_US` | A3 / 5000 | Hall ADC/주기 |
| `HALL_ADC_SAMPLES`, `HALL_BASELINE_SAMPLES`, `HALL_BASELINE_SAMPLE_DELAY_US` | 8 / 64 / 250 | Hall 평균 및 부팅 기준값 |
| `HALL_BASELINE_TRACK_DIVISOR` | 128 | 자석 없는 구간 기준값 느린 추적 |
| `HALL_TRIGGER_DELTA_ADC`, `HALL_RELEASE_DELTA_ADC` | 30 / 12 | 기준 ADC 차이의 감지/해제 문턱 |
| `HALL_MIN_PULSE_INTERVAL_US`, `HALL_ZERO_TIMEOUT_US` | 20000 / 5000000 | 잡음 중복 펄스 억제/측정 유효기간 |
| `HALL_PULSES_PER_WHEEL_REV` | 12 | 펄스→속도 환산 |
| `HALL_CALIBRATED`, `REQUIRE_HALL_CALIBRATION_FOR_ARM` | true / false | 보정 플래그/ARM 필수 여부; false여도 피드백·고장 감시 켜짐 |
| `HALL_STALL_TARGET_MIN_MRAD_S`, `HALL_STALL_TIMEOUT_MS` | 20 / 5000 | 스톨 감시 목표 문턱/최소 시간 |
| `HALL_STALL_EXPECTED_PULSE_PERIODS`, `HALL_STALL_MAX_TIMEOUT_US` | 2.5 / 10000000 | 저속 적응형 스톨 기한 |
| `HALL_MAX_PLAUSIBLE_MRAD_S`, `HALL_OVERSPEED_CONFIRM_PULSES` | 25000 / 2 | Hall 비현실적 속도 fault |
| `MOTOR_PWM_PIN`, `MOTOR_IN1_PIN`, `MOTOR_IN2_PIN`, `MOTOR_SIGN` | 5 / 6 / 8 / +1 | 실제 출력 핀/배선 부호 |
| `MAX_PWM` | 140 | Arduino count 상한 |
| `MOTOR_FF_BIAS_PWM`, `MOTOR_FF_NOMINAL_PWM`, `MOTOR_START_PWM` | 30 / 60 / 20 | FF bias/1m/s 예측값/초기 step |
| `MOTOR_NOMINAL_MRAD_S` | 1/0.115×1000 | FF 모델 기준 각속도 |
| `MOTOR_PWM_RISE_PER_S`, `TERRAIN_RECOVERY_PWM_RISE_PER_S`, `MOTOR_PWM_FALL_PER_S` | 20 / 10 / 60 | PWM 상승/복귀 상승/하강 제한 |
| `MOTOR_REVERSAL_BRAKE_US` | 150000 | 역전 전 BRAKE 시간 |
| `SPEED_BRAKE_ABSOLUTE_MRAD_S`, `SPEED_BRAKE_RELEASE_MRAD_S`, `SPEED_BRAKE_CONFIRM_PULSES` | 8km/h / 7km/h 각속도 환산 / 2 | 절대 과속 BRAKE/해제/확인 |
| `MOTOR_PID_KP`, `MOTOR_PID_KI`, `MOTOR_PID_KD`, `PID_INTEGRAL_LIMIT` | 12 / 0 / 0 / 30 | 피드백 게인/적분 상태 제한 |
| `VELOCITY_FILTER_ALPHA` | 0.35 | 새 Hall 펄스에서 속도 EMA |
| `REQUIRE_DEADMAN`, `PRESSURE_LEFT_PIN`, `PRESSURE_RIGHT_PIN` | true / A2 / A1 | 양손 허용조건/입력 |
| `PRESSURE_SAMPLE_PERIOD_MS`, `PRESSURE_FILTER_ALPHA`, `PRESSURE_ADC_SAMPLES` | 100 / 0.2 / 8 | 압력 주기/필터/평균 |
| `PRESSURE_PRESENT_HYSTERESIS`, `PRESSURE_RELEASE_DEBOUNCE_SAMPLES` | 3 / 2 | 손 접촉 해제 문턱/연속 샘플 |
| `PRESSURE_LEFT_ACTIVE_HIGH`, `PRESSURE_RIGHT_ACTIVE_HIGH` | true / true | 눌렀을 때 ADC 증가 |
| `PRESSURE_LEFT_PRESENT_THRESHOLD`, `PRESSURE_RIGHT_PRESENT_THRESHOLD`, `PRESSURE_THRESHOLDS_CALIBRATED` | 35 / 35 / true | 접촉 문턱/보정 표시 |
| `PRESSURE_IMBALANCE_THRESHOLD`, `PRESSURE_SUDDEN_CHANGE_THRESHOLD` | 300 / 150 | 불균형/급변 WARNING; 자체 속도 감소 없음 |
| `MAGNET_BENCH_MODE`, `MAGNET_BENCH_PWM`, `MAGNET_BENCH_PULSE_HOLD_MS` | false / 60 / 750 | 별도 자석 시험 모드/고정 출력/허용 창 |
| `MAGNET_BENCH_VELOCITY_HOLD_US`, `MAGNET_BENCH_VELOCITY_FILTER_ALPHA` | 5000000 / 1.0 | 시험 모드 속도 관측 유지/필터 |
| `ENABLE_ESTOP`, `ESTOP_PIN`, `ESTOP_ACTIVE_LEVEL` | false / 12 / HIGH | 비활성 E-stop |
| `USE_DRIVER_FAULT_PIN`, `DRIVER_FAULT_PIN`, `DRIVER_FAULT_ACTIVE_LEVEL` | false / 13 / LOW | 비활성 드라이버 fault |
| `ENABLE_BATTERY_SENSE`, `BATTERY_SENSE_PIN`, `ADC_REFERENCE_V`, `BATTERY_DIVIDER_RATIO` | false / A5 / 5.0 / 3.0 | 비활성 배터리 telemetry |
| `ENABLE_CURRENT_SENSE`, `LEFT_CURRENT_SENSE_PIN`, `RIGHT_CURRENT_SENSE_PIN`, `CURRENT_ZERO_V`, `CURRENT_MA_PER_V` | false / A0 / A4 / 2.5 / 1000 | 비활성 전류 telemetry |
| `ENABLE_FRONT_RANGE_SENSORS` | false | 전방 센서 capability 미광고 |
| `AVR_BOOT_COUNTER_EEPROM_ADDRESS` | 0 | 부팅 ID 영속 저장 |

## 9. Hall 센서 내부 함수와 변수

소스: `firmware/safestride_mcu/analog_hall_sensor.h/.cpp`.

| 함수 | 설명 |
|---|---|
| `AnalogHallSensor()`, `begin()` | 상태 초기화; 부팅 시 64회 ADC 평균을 무자기장 기준으로 사용 |
| `update()` | 5ms 주기 ADC 평균 → 기준 차이 → 진입/해제 히스테리시스 → 펄스/period 갱신 |
| `readAveraged()` | 8회 ADC 평균 |
| `magnitude()` | ADC와 기준의 차이 절댓값; 자석 양극성 모두 감지 |
| `pulseCount()`, `periodUs()`, `ageUs()` | count/period/age 반환; 아직 펄스 없으면 age=`0xFFFFFFFF` |
| `rawAdc()`, `baselineAdc()`, `magnetPresent()` | ADC/기준/자석 감지 상태 조회 |

| 변수 | 설명 |
|---|---|
| `pulse_count_`, `last_pulse_us_`, `period_us_`, `last_sample_us_` | 누적 펄스, 펄스시각, 두 펄스 주기, ADC 샘플시각 |
| `baseline_q8_`, `raw_adc_`, `magnet_present_` | 소수 8bit 기준 ADC, 실측 ADC, 자석 진입 유지 상태 |
| `baseline_adc`, `delta_adc`, `elapsed_us`, `sample_q8`, `total` | 기준 차이/잡음 시간창/기준 추적/평균 계산 지역값 |

첫 펄스는 위치만 갱신한다. 두 번째 펄스부터 속도 계산이 가능하다.
자석이 감지된 동안은 추가 계수하지 않고, 기준 차이가 12ADC 이하로 돌아와야 재감지 가능하다.
필터는 새 펄스에서만 `filtered += 0.35×(raw-filtered)`로 갱신된다.

## 10. 압력 센서 내부 함수와 변수

소스: `firmware/safestride_mcu/pressure_sensor.h/.cpp`.

| 함수 | 설명 |
|---|---|
| `PressureSensorPair()`, `begin()` | 원시 ADC/필터/접촉/경고 상태 초기화 |
| `update()`, `sample()` | 100ms마다 양쪽 측정·필터·접촉·불균형/급변 계산 |
| `readAveraged()` | ADC mux 전환 후 첫 측정 폐기, 8회 평균 |
| `filterChannel()` | EMA 0.2; 기존 접촉에서 해제 raw가 나오면 필터를 raw로 바꾸어 잔류값 제거 |
| `channelPresent()` | 접촉 진입은 raw와 필터 모두 35 이상, 유지/해제는 raw와 3ADC 히스테리시스 사용 |
| `updatePresence()`, `updateChannelPresence()` | 양쪽 접촉 상태 업데이트; 해제 2개 연속 샘플 필요 |
| `bothHandsPresent()`, `leftPresent()`, `rightPresent()` | 실제 주행 허용조건에 사용 |
| `initialized()`, `calibrated()`, `alert()` | 초기화/보정/진단 상태 |
| `leftRaw()`, `rightRaw()`, `leftFiltered()`, `rightFiltered()`, `difference()`, `maximumDelta()` | telemetry/경고용 수치 조회 |

| 변수 | 설명 |
|---|---|
| `initialized_`, `last_sample_ms_` | 초기화/측정 주기 |
| `left_raw_`, `right_raw_`, `left_`, `right_` | 양쪽 raw/필터값 |
| `previous_left_`, `previous_right_`, `difference_`, `maximum_delta_` | 이전값, 좌우 차이, 최대 변화량 |
| `left_present_`, `right_present_`, `left_release_samples_`, `right_release_samples_` | 접촉/해제 연속 샘플 상태 |
| `alert_`, `PressureAlert::NORMAL/WARNING/HANDS_OFF` | 정상/불균형 또는 급변/손 해제 진단 |
| `raw_left/right`, `left_delta/right_delta`, `sample_present`, `was_present`, `active_high`, `threshold`, `hysteresis`, `alpha` | 접촉/필터/경고 계산 입력 및 지역값 |

해제 확정에는 샘플 타이밍에 따라 약 100~200ms가 걸리고, 이후 MCU의 600ms 감속이 시작된다.
그보다 높은 우선순위의 통신 기한/고장이 발생하면 즉시 정지한다.

## 11. MPU6050 내부 함수·변수·설정

소스: `firmware/terrain_mcu/mpu6050_sensor.h/.cpp`, `firmware/terrain_mcu/config.h`.

| 함수 | 설명 |
|---|---|
| `Mpu6050Sensor()`, `begin()` | 센서/자세/valid 상태 초기화, I2C 설정 |
| `update()` | 50ms마다 14byte 읽기, 가속도/자이로 단위 변환, roll/pitch 계산 |
| `configure()`, `probe()` | `0x68/0x69` 탐색, ID 확인, 측정범위/샘플속도/필터 설정 |
| `writeRegister()`, `readRegisters()` | I2C 레지스터 쓰기/읽기 |
| `noteReadFailure()` | 1회 실패도 valid=false, 연속 3회면 재설정 경로 |
| `valid()`, `address()` | 상태/주소 조회 |
| `accelXMg/YMg/ZMg()` | 축 가속도 telemetry |
| `gyroXMradS/YMradS/ZMradS()` | 축 자이로 telemetry; 속도 피드백/경사 계산에는 미사용 |
| `rollMrad()`, `pitchMrad()` | 필터 각도 반환; 직접 경사 속도 정책에 들어가는 값은 pitch |
| `readBigEndianI16()`, `roundedI16()` | 레지스터 정수 해석/포화 반올림 |
| `shortestAngleDeltaMrad()` | roll의 ±π 경계에서 짧은 방향의 차이로 필터링 |

```text
pitch = atan2(-ax, sqrt(ay²+az²)) × 1000    [mrad]
roll  = atan2(ay, az) × 1000               [mrad]
pitch_filtered += 0.15 × (pitch_new-pitch_filtered)
```

| 변수/상수 | 역할/값 |
|---|---|
| `configured_`, `valid_`, `attitude_initialized_`, `address_`, `consecutive_errors_` | 연결·측정·필터 초기화·주소·오류 상태 |
| `last_sample_ms_`, `last_reconnect_ms_` | 샘플/재연결 스케줄 |
| `accel_x/y/z_mg_`, `gyro_x/y/z_mrad_s_`, `roll_mrad_`, `pitch_mrad_` | 저장 측정/각도 |
| `raw_ax/ay/az`, `raw_gx/gy/gz`, `ax/ay/az`, `roll`, `pitch`, `sample` | 레지스터 및 자세 계산 지역값 |
| `ENABLE_MPU6050` | true |
| `MPU6050_ADDRESS_LOW/HIGH` | 0x68/0x69 |
| `MPU6050_SAMPLE_PERIOD_MS`, `MPU6050_SAMPLE_RATE_DIVIDER` | 50 / 49; 20Hz 출력/읽기 |
| `MPU6050_RECONNECT_PERIOD_MS`, `MPU6050_MAX_CONSECUTIVE_ERRORS` | 1000 / 3 |
| `MPU6050_ATTITUDE_ALPHA` | 0.15 |
| `REG_SMPLRT_DIV`, `REG_CONFIG`, `REG_GYRO_CONFIG`, `REG_ACCEL_CONFIG`, `REG_ACCEL_XOUT_H`, `REG_PWR_MGMT_1`, `REG_WHO_AM_I`, `WHO_AM_I_MPU6050` | 센서 구성/식별/데이터 레지스터 및 ID |
| `PI_MRAD`, `GYRO_RAW_TO_MRAD_S` | 각도 경계 및 자이로 단위 환산 |

현재 가속도를 중력 방향으로 가정하므로 주행 가감속/충격도 pitch에 섞일 수 있다. 자이로와 융합한 경사 추정은 구현되어 있지 않다.

## 12. ToF 내부 함수·변수·설정

소스: `firmware/terrain_mcu/tof10120_sensor.h/.cpp`, `firmware/terrain_mcu/config.h`.

| 함수 | 설명 |
|---|---|
| `Tof10120Sensor()`, `begin()` | 거리/필터/기준/후보/위험 상태 초기화 |
| `update()` | 50ms마다 읽기; 100~2000mm 밖/통신 실패면 valid=false |
| `readDistanceI2c()` | 거리 레지스터에서 2byte 읽기; 실패 0xFFFF |
| `classify()` | EMA → 고정 기준 차이 → 같은 방향 4회 확정 → 위험 1초 hold |
| `valid()`, `distanceMm()`, `filteredDistanceMm()`, `referenceDistanceMm()`, `errorMm()`, `changeMm()`, `alert()` | 유효성/거리/기준/오차/변화/위험 단계 조회 |

| 변수/상수 | 역할/값 |
|---|---|
| `initialized_`, `valid_`, `baseline_count_`, `last_sample_ms_` | 초기화/10샘플 warmup/최근 측정 |
| `distance_mm_`, `filtered_mm_`, `reference_mm_`, `error_mm_`, `change_mm_` | raw/필터/고정 지면 기준/기준 차이/직전 필터 차이 |
| `candidate_direction_`, `consecutive_count_` | 낙차(+1)/돌출(-1) 후보 및 연속 횟수 |
| `red_hold_active_`, `last_red_ms_`, `alert_`, `last_hazard_alert_` | 위험 유지 여부/마지막 확정시각/현재 및 최근 확정 위험 |
| `TofAlert::NORMAL/CANDIDATE_RAISED/CANDIDATE_DROP/RAISED/DROP/INVALID` | 정상/후보/확정/무효 분류 |
| `TOF_I2C_ADDRESS`, `TOF_DISTANCE_REGISTER`, `TOF_SAMPLE_PERIOD_MS` | 0x52 / 0x00 / 50 |
| `TOF_MIN_VALID_DISTANCE_MM`, `TOF_MAX_VALID_DISTANCE_MM` | 100 / 2000 |
| `TOF_FILTER_ALPHA` | 0.3 |
| `TOF_GROUND_DISTANCE_MM` | 442×√2 ≈625.1mm; 442mm 높이/45° 광선 기준 |
| `TOF_ERROR_THRESHOLD_MM` | 250×√2 ≈353.6mm; 수직 250mm 차이의 광선 거리 환산 |
| `TOF_BASELINE_SAMPLES`, `TOF_REQUIRED_FRAMES`, `TOF_RED_HOLD_MS` | 10 / 4 / 1000 |
| `previous_filtered`, `direction`, `same_direction`, `distance`, `distance_mm` | 오차/필터/연속 판정 지역값 |

기준 지면은 현재 **고정값**이다. 초기 10개는 필터 예열이지 장애물을 정상 지면으로 학습하는 과정이 아니다.
`change_mm_`는 현재 분류 확정 조건에 직접 사용하지 않는다. 오차 문턱과 같은 방향 연속 횟수로 판정한다.
현재 ToF 제어는 비활성이므로 무효 또는 확정 위험 모두 모터 명령을 바꾸지 않는다. 기능 활성화 시 무효 ToF는 신규 감속을 시작하지 않고 기존 위험의 정상 복귀 증거도 아니다.

## 13. Terrain 보드 및 브리지의 전달 함수/상태

소스: `firmware/terrain_mcu/terrain_mcu.ino`, `src/safestride_bridge/safestride_bridge/terrain_bridge_node.py`.

- Terrain `setup()/loop()`가 ToF/MPU를 주기 갱신하고 `sendTelemetry()`로 측정·valid·alert·fault를 전송한다.
- `sendTelemetry()` 내부의 `faults`가 ToF/MPU 무효 상태를 fault bit로 운반한다. `roundedUnsigned16()/roundedSigned16()`은 거리/오차 송신 수치를 포화 반올림한다.
- `sendHello()/processHostProtocol()/makeBootId()`는 보드 광고/세션 시작 프레임/영속 부팅 ID를 관리한다. `CAP_TOF10120/CAP_MPU6050`, `FAULT_TOF_INVALID/FAULT_MPU_INVALID`는 기능/무효 표시 bit다.
- `g_tof`, `g_mpu`가 센서 상태, `g_session_active`, `g_boot_id`, `g_session_id`, `g_last_telemetry_ms`, `g_last_hello_ms`, `g_tx_sequence`, `g_receiver`가 연결/송신 상태다.
- `g_display/g_lcd/g_last_display_status_ms`와 `sendDisplayStatus()`는 HMI 표시/연결 상태 전달용이며 직접 모터 제어에는 사용하지 않는다.
- Terrain 설정 `SERIAL_BAUD=115200`, `HELLO_PERIOD_MS=500`, `TELEMETRY_PERIOD_MS=50`, `AVR_BOOT_COUNTER_EEPROM_ADDRESS=8`이 전달/연결에 관여한다. `SESSION_LOSS_TIMEOUT_MS=1500`은 현재 Terrain sketch에서 세션 만료 검사에 사용하지 않는 선언이다. 브리지/감독은 별도로 telemetry 나이를 검사한다.
- `TerrainBridgeNode._io_tick()/_handle_frame()/_handle_hello()`는 프레임/순번/세션/규약을 확인하고 최신 측정을 저장한다.
- `_publish_status()`가 mm→m, mrad→rad 환산 후 `/terrain/status`의 ToF/pitch/valid/fault/나이를 발행한다. 이 토픽을 SafetySupervisor가 소비한다.
- `_publish_telemetry()`는 ToF Range, `_publish_imu()`는 IMU 메시지 발행. 감독은 IMU 토픽보다 TerrainStatus의 `pitch_rad`를 직접 사용한다.
- `_gravity_compensated_acceleration()`, `_quaternion_from_rpy()`는 IMU 표시용 중력 보정/자세 변환; 감독의 pitch FF/PWM을 직접 계산하지 않는다.
- `__init__()/_declare_parameters()/_load_parameters()/_value()`는 연결 구성, `_now()/_connected()/_link_ok()`는 시각/연결 판단, `_reset_link()/_close_serial()/_try_open_serial()`은 재연결, `_next_sequence()/_write_frame()`은 송신, `_diagnostic_tick()`은 표시, `destroy_node()/main()`은 수명 관리다.
- `_last_telemetry`, `_last_telemetry_time`, `_last_telemetry_sequence`, `_session_id`, `_boot_id`, `_capabilities`, `_session_started`, `_serial`, `_parser`, timeout/reconnect 설정이 최신 경사/위험 입력 전달에 관여한다.
- HMI의 `pitch_sign/pitch_offset_rad`는 화면 경사 표시 보정이다. 실제 속도 정책의 보정은 SafetySupervisor 파라미터다.

## 14. 메시지와 바이너리 통신

소스: `src/safestride_interfaces/msg/`, `firmware/safestride_mcu/protocol.h/.cpp`, `src/safestride_bridge/safestride_bridge/protocol.py`.

| 데이터 | 속도 관련 필드/의미 |
|---|---|
| `TwistStamped /cmd_vel` | `twist.linear.x`, `twist.angular.z`, `header` 원래 목표/나이 |
| `TwistStamped /cmd_vel_safe` | 감독 후 목표; 현재 브리지의 실제 입력은 아래 DriveCommand |
| `DriveCommand` | `target_linear_m_s`, `slope_ff_pwm`, `drive_pwm_cap`, `mode`, `header` |
| `DriveCommand.target_speed_kmh` | 표시용; 제어는 `target_linear_m_s` 사용 |
| 명령 `mode` | DRIVE=0, BRAKE=1, TERRAIN_STOP=2 |
| `CommandPayload` | `target_mrad_s`, `ttl_ms`, `enable`, `reserved`, `slope_ff_pwm`, `drive_pwm_cap`, `mode`; `<iHBBhBB`, 12byte |
| `Frame/FrameView` | type/flags/sequence/payload_length/session_id/timestamp/payload; 올바른 새 세션 명령인지 검사 |
| `TelemetryPayload` | hall 좌우 count/velocity, status_bits, fault_bits, last_command_sequence, 양손 raw/filter/flags/alert, ff_pwm, feedback_pwm, applied_pwm, speed_age_us, speed_flags, drive_mode |
| telemetry `drive_mode` | 실제 BRAKE(1)/구동(0) 표시; 명령 TERRAIN_STOP(2)와 달리 0/1 값만 운반 |
| `WheelHall` | 좌우 펄스/각속도/kmh/calibrated/센서 존재; 오른쪽은 독립 센서가 아님 |
| `WalkerStatus` | state/link_ok/armed/estop/watchdog/deadman/fault/telemetry_age가 감독 허용조건; PWM/측정속도/age/valid/new_pulse/braking은 제어 관측 |
| `WalkerStatus.direction_valid` | 항상 false; 실제 방향 센서 없음 |
| `TerrainStatus/TerrainTelemetryPayload` | ToF raw/filtered/reference/error/change/valid/alert, pitch/roll, mpu_valid, fault_bits, telemetry_age |
| `SurfaceCondition` | classification/confidence/recommended_speed_scale/valid/model_version; 현재 속도 반영 꺼짐 |

- MCU `FrameReceiver::push()/decode()/reset()`, `crc16CcittFalse()`, `sequenceIsNewer()`, `sendFrame()`, `readU16/I16/U32/I32()`, `writeU16/I16/U32/I32()`가 프레임 검증/직렬화를 담당한다.
- Python `FrameParser.feed()/reset()`, `decode_frame()/encode_frame()`, `Frame.raw_bytes()/encode()/__post_init__()`, `crc16_ccitt_false()`, `sequence_is_newer()`, `cobs_encode()/cobs_decode()`가 같은 기능을 담당한다.
- `CommandPayload/TelemetryPayload/TerrainTelemetryPayload/HelloPayload/SessionStartPayload.pack()/unpack()`, `_require_size()`, `_u8/_u16/_u32()`는 payload 크기/수치 범위를 검증한다.
- `VERSION=6`, `SCHEMA_ID=0x0601`, `FIRMWARE_RELEASE_ID=20260908`, 보드 role, `TYPE_*`, payload/frame 크기, CRC 및 COBS delimiter가 호환되지 않으면 정상 명령 스트림이 형성되지 않아 출력이 차단된다.
- `ProtocolError` 및 decode/CRC/version/payload 예외, `ReceiveResult::FRAME_READY/FRAME_ERROR/CRC_ERROR`는 유효 프레임만 제어에 전달하도록 한다.
- `WalkerStatus`에 예약된 OVERCURRENT/UNDERVOLTAGE/OVERTEMPERATURE bit가 있다는 사실만으로 해당 센싱/제어가 구현되었다고 볼 수 없다.

## 15. 현재 비활성인 선택 속도 정책

### 카메라 노면 제어

소스: `src/safestride_perception/safestride_perception/surface_policy.py`, `surface_perception_node.py`.

- `SurfacePerceptionNode._open_camera()/_close_camera()`는 입력 연결, `_frame_tensor()`는 전처리, `_infer()`는 추론/예측 EMA, `_timer_callback()`는 프레임 수집·분류·배율 결정, `_publish_surface()`는 권장 배율 발행이다.
- `prediction_is_confident()`는 top1 ≥0.65 및 top1-top2 ≥0.15 조건을 확인한다. `speed_scale()`는 label을 `SCALES` 배율로 변환한다.
- 현재 정책 배율: smooth/rough/mud/unpaved 1.0, block/gravel 0.95, wet 0.85, snow_ice 0.75, step/hole 0.0. 알 수 없는 label은 0.0 반환한다.
- 노드의 모델/클래스 경로, confidence threshold, top1 margin, EMA alpha(0.75), inference rate(1Hz), 카메라/최근 프레임/확률 상태가 배율 생성에 관여한다.
- `__init__()/_finite_parameter()/_now_seconds()`는 구성·검증·시각, `_publish_preview()/_maybe_publish_diagnostics()`는 관측, `destroy_node()/main()`은 수명 관리다.
- 감독의 `surface_control_enabled=false`이므로 현재 분류값으로 모터 속도를 바꾸지 않는다.

### GPS·횡단보도 속도 제어

소스: `src/safestride_navigation/safestride_navigation/crosswalk_controller_node.py`, `crossing_policy.py`, `speed_profile.py`, `gps_motion.py`, `src/safestride_sensors/safestride_sensors/gps_speed_filter.py`.

- 현재 `motion_output_enabled=false`로 `/cmd_vel` 발행은 꺼져 있다. 기본 모터 목표는 CruiseCommandNode가 만든다.
- `CrosswalkController._fix_callback()/_gps_speed_callback()/_gps_course_callback()/_odom_callback()`는 GPS/바퀴 상태 저장, `_measured_motion()`과 `select_motion_measurement()`는 사용할 측정속도 선택이다.
- `_wheel_motion_active()/_motion_confirmed()/_fresh()/_heading()`는 바퀴 움직임/최근성/방향 확인, `_tick()`은 지도/신호/보행 프로필/상태기계를 종합한다.
- `_signal_state()/_request_signal_if_due()/_consume_signal_future()`와 교차로 지도 요청·수신 함수가 남은 신호시간/교차로를 제공한다. 지도·신호 파싱/거리·방향 helper는 상위 정책 입력 생성이지 MCU 속도 피드백 함수가 아니다.
- `_publish_command()`는 선택 정책을 켰을 때 최종 요청 속도를 `/cmd_vel`로 전달한다. `_publish_status()/_publish_diagnostic()`는 판단 결과 표시다.
- `UserSpeedProfile.add()/load()/save()`는 0.12~1.8m/s의 최대 300개 표본을 유지/영속화. `safe_speed()`는 `percentile(samples, 0.20)`을 0.30~1.00m/s로 제한; 표본 없으면 기본 0.50m/s.
- 프로필 변수 `path`, `default_speed_mps`, `samples`; 횡단보도 변수 `_profile`, `_controller`, `_motion_output_enabled`, 측정속도/나이, 위치/방향/바퀴 거리, active crossing, 신호시간, `desired_speed/effective_speed`가 상위 요청에 관여한다.
- `GpsMotionTracker.update()/set_course()/heading()/coordinates_stuck()`는 위치/방향/좌표 정지 판단; `allow_gps_speed_fallback=false`이면 GPS만으로 바퀴 움직임을 대신 확인하지 않는다.
- `GpsSpeedFilter.update()/reset()`는 GPS 위치 변화/수신기 속도/품질/방향 일관성/확정 횟수/EMA를 사용해 GNSS drift를 제외한다. `_distance_m()/_course_coherence()/_optional_float()`는 거리/방향/수치 보조 함수다.
- GPS 노드 `__init__()/_connect()/_close()/_poll()/_publish_sentence()`는 연결/문장 수신/위치·속도 발행과 필터 갱신, `_fresh_quality()/_age()/_now()`는 품질 최근성/시각, `_publish_diagnostic()/_format_float()`는 상태 표시다.
- GPS 필터 파라미터 전체는 실행 YAML의 `gps_node.speed_filter_*`에 있다: window/settling/minimum span/sample/displacement/HDOP scale/path efficiency/course coherence/speed agreement/max HDOP/min satellites/quality/enter-exit confirmations/smoothing/max speed. 이들은 GPS 속도 추정에 관여하며 현재 MCU P 입력에는 들어가지 않는다.

`CrossingStateMachine`의 함수: `__init__()/set_state()/lock()/reset()/current_crosswalk()/_record_progress()/_automatic_start_detected()/update()/command()`.
상태 변수: `parameters`, `_clock`, `state`, `state_since`, `locked_crosswalk`, `locked_intersection_id`, `progress_history`, `exit_seen_since`, `crossing_started_at`, `arm_wheel_origin`, `crossing_wheel_origin`, `reason`.

| 횡단 상태 | 생성 가능한 요청 |
|---|---|
| IDLE | 프로필 안전속도 |
| APPROACHING | 안전속도와 0.50m/s 중 작은 값 |
| WAIT_AT_CURB | 0 |
| ENTRY_ALLOWED | 안전속도 |
| CROSSING | 안전속도와 측정속도 중 큰 값 |
| CROSSING_URGENT | 안전속도+0.10/측정속도 중 큰 값을 최대 보조속도로 제한 |
| EXITING | 안전속도와 0.50m/s 중 작은 값 |

모든 요청은 `maximum_assist_speed_mps=0.85`로 최종 제한된다.
`CrossingParameters`에는 접근/잠금/연석 구역 거리, 이탈 거리, 진입 진행량·증가량·시간창·최저속도,
출구 clearance/hold, 완료 hold, 횡단 timeout, reaction/entry margin/crossing margin,
minimum estimate speed/maximum assist speed가 있어 상태 및 요청속도 결정을 바꾼다.
선택 횡단보도 출력을 켜고 Cruise도 켜면 둘 다 같은 `/cmd_vel`에 발행할 수 있다. 현재 코드에는 이 두 상위 목표 사이의 전용 mux가 없다.

## 16. 표시 전용 함수와 실행 설정

- `SpeedDisplay.__init__()/_publish()/main()`은 `/gps/speed`, `/gps/speed_raw`, `/odom`, `/cmd_vel`, `/cmd_vel_safe` 값을 ×3.6 하여 `_kmh` 토픽으로 발행한다. PWM이나 목표속도를 수정하지 않는다.
- HMI relay/model/Lua와 Foxglove는 목표/측정/PWM/경사/위험 표시를 담당한다. 현재 화면의 표시용 km/h/pitch 값을 MCU 속도 P에 되먹이지 않는다.
- `scripts/run.sh`: `SAFESTRIDE_CONFIG`, `SAFESTRIDE_WHEEL_RADIUS_M`, `SAFESTRIDE_WHEEL_SEPARATION_M`, `SAFESTRIDE_ENABLE_CRUISE`, `SAFESTRIDE_ENABLE_CROSSWALK`, `SAFESTRIDE_REQUIRE_SURFACE_CONDITION` 등 환경변수가 사용 설정/노드/기구값을 선택한다.
- `safestride.launch.py.generate_launch_description()`의 `config_file`, `wheel_radius`, `wheel_separation`, `enable_cruise`, `enable_crosswalk`, perception/terrain/GPS 관련 launch argument와 `ParameterValue`가 각 노드의 실제 설정을 결정한다.
- 기본 실행은 `config/raspberry_pi.yaml`, 직접 launch 기본은 패키지 `src/safestride_bringup/config/safestride.yaml`다. 런타임 덮어쓰기까지 있어야 실제 적용값을 확정할 수 있다.

## 17. 읽는 순서

1. `cruise_command_node.py`: 기본 요청 속도.
2. `safety_supervisor_node.py::_timer_callback()`, `safety_logic.py`: 안전 목표/경사/정지 모드.
3. `serial_bridge_node.py::_command_tick()`: m/s→mrad/s, TTL 및 enable.
4. `safestride_mcu.ino::handleCommand()/runControlLoop()`: 물리조건/수신/호출.
5. `motor_control.cpp::update()`: 실제 FF+P/PWM/정지 계산.
6. `analog_hall_sensor.cpp`, `pressure_sensor.cpp`, Terrain 센서: 측정값 생성.
7. 두 설정 YAML과 두 보드 `config.h`: 현재 수치와 선택 경로.

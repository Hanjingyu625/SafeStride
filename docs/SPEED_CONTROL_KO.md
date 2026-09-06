# SafeStride 속도제어 — pdj1

작성일: 2026-09-06. 최신 `pdj` 커밋 `d7be094`에서 분기한 구현이다.
이 문서는 코드 동작과 초기 튜닝값을 설명한다. 실제 지면 속도·정지거리·경사 위
정지 유지 성능은 아직 측정하지 않았다.

## 1. 제어 흐름

```text
Terrain Uno MPU6050 → USB → terrain bridge → /terrain/status
                                                 ↓
/cmd_vel → safety supervisor → /drive/command (DriveCommand)
                                   ↓
                              serial bridge
                                   ↓ USB protocol v5
Drive Uno: 목표 ramp → FF + Hall P → 방향/상한 제한 → PWM slew → 모터드라이버
                ↑                                       ↓
          A3 WSH135, 6 pulse/rev                   모터 두 개 공통 구동
```

`/cmd_vel_safe`는 감독된 목표속도를 관찰하기 위한 `TwistStamped`로 유지한다.
실제 bridge 입력은 `/drive/command`이며 목표속도·경사 FF·상한·mode가 한 메시지다.
속도와 경사 보정을 별도 토픽으로 보내 이전 보정이 새 속도에 섞이게 하지 않는다.

주요 코드:

- `firmware/safestride_mcu/config.h`, `motor_control.cpp`: FF, Hall, PWM, BRAKE.
- `src/safestride_control/safestride_control/safety_logic.py`: 경사 분류와 BRAKE 복구.
- `safety_supervisor_node.py`: 센서 freshness, 속도 제한, 원자 명령 발행.
- `src/safestride_bridge/safestride_bridge/serial_bridge_node.py`: 명령 검증·직렬 전송.
- `config/raspberry_pi.yaml`, `src/safestride_bringup/config/safestride.yaml`: 실행 설정.

## 2. 평지 출력과 속도 피드백

PWM은 Arduino `analogWrite()`의 **0~255 count**다. 60은 60%가 아니라 약 23.5%
duty다. 30은 약 11.8%, 현재 상한 100은 약 39.2%다.

```text
wheel_radius = 0.115 m
v_nom = 0.08 m/s
omega_nom = 0.08 / 0.115 ≈ 0.696 rad/s

u_ff = sign(omega_ref) × [30 + (60 - 30) × abs(omega_ref) / omega_nom]
u_P  = 12 × (omega_ref - omega_measured)     # rad/s 단위
u_raw = u_ff + u_slope + u_P
```

목표가 0이면 BRAKE다. 매우 작은 ramp 목표(|목표| < 20 mrad/s)의 FF도 0이다.
`30`은 계산 bias이며 출력 하한이 아니다. 전진 목표에서 출력은 0~cap, 후진에서는
-cap~0으로 제한하므로 감속 중 30 아래나 0까지 내려갈 수 있다. 속도 오차만으로
목표 반대 방향 PWM을 만들지 않는다. Ki=Kd=0을 코드 assertion으로 고정했다.

평지에서 목표·측정 속도가 모두 0.08 m/s이면 모델 출력은 약 60이다.
이는 실측 보정 전 초기 기준이며, 부하에 따라 P 항이 출력을 수정한다.
기존 `MOTOR_FEEDFORWARD=10`과 deadzone bias 중복 덧셈은 제거했다.

정상 PWM 크기는 상승 20 count/s, 하강 60 count/s로 제한한다. 예를 들어 출력
0→60은 목표·오차가 충분하더라도 약 3초 이상 걸린다. Pi 목표속도 slew 및 MCU
목표각속도 ramp와 최종 PWM slew는 별개다. 새 cap은 즉시 지킨다.
BRAKE·fault·watchdog은 정상 slew를 기다리지 않는다.

## 3. 경사 보정

정규화는 `(pitch_rad - pitch_offset_rad) * uphill_pitch_sign` 순서다.
현재 offset `-0.196164 rad`, sign `+1`을 유지했다. 장착 변경 시 평지에서 재측정한다.

경사 상태 진입 5°, 이탈 3°, 확인 0.5초의 히스테리시스를 유지한다.
오르막 목표속도 배율은 **1.0**이다. 내리막 확정 상태의 배율은 다음과 같다.

```text
amount = clamp((-normalized_pitch_deg - 3) / 5, 0, 1)
downhill_scale = 1 - 0.4 × amount
```

3°에서 1.0, 6°에서 0.76, 8° 이상에서 0.6이다. 3~5° 구간은 상태 이력에 따라
평지 또는 내리막일 수 있다. 노면 제어는 현재 꺼져 있다.

| 확정 상태 | 경사 FF (PWM count) |
|---|---|
| 평지 | 0 |
| 오르막 | `min(30, max(0, 4 × (각도 - 3)))` |
| 내리막 | `-min(60, max(0, 6 × (-각도 - 3)))` |

예: +5°는 +8, -5°는 -12. 실제 출력에는 감속된 목표속도에 따른 FF와 P도 더해진다.
후진 명령에는 전진 기준 경사 FF를 적용하지 않는다. MPU의 현재 pitch는 가속도
기반 추정이므로 가감속·충격 영향을 받는다. 위 값은 검증된 보행 안전 기준이 아니다.

## 4. BRAKE 유지와 자동 복귀

사용자가 지정한 [DFRobot DRI0042 제어 자료](https://wiki.dfrobot.com/dri0042/#tech_specs)를
기준으로 `IN1=LOW, IN2=LOW`를 BRAKE로 사용한다. `HIGH/HIGH`는 Vacant이며 사용하지 않는다.
PWM 핀은 0으로 두지만 보드 전원은 유지하고 BRAKE 핀 상태를 계속 유지한다.
전원 릴레이 차단이나 새 수동 재시작 잠금을 추가하지 않았다.

참고 자료는 **DRI0042**의 문서이며 장착품 명칭은 **SZH-GNP521**이다.
장착 보드 리비전의 일치와 실제 BRAKE 효과는 실물 확인 대상이다.
방향 전환에는 자료의 0.1초 초과 권고를 반영해 최소 150ms BRAKE를 비차단 방식으로 둔다.
역방향 PWM을 제동으로 사용하지 않는다.

| 조건 | 동작·복귀 |
|---|---|
| 정규화 pitch ≤ -10° | 즉시 BRAKE. -7° 이상이 유효하게 0.5초 유지되면 자동 DRIVE 복귀 |
| MPU invalid, fault, nonfinite 또는 stale | BRAKE 명령 스트림 유지. 유효 복구 조건에서 자동 재개 |
| 실제 속도 추정 > 요청 +0.05 m/s 또는 >0.18 m/s가 0.2초 지속 | MCU BRAKE |
| 과속 후 속도 추정 < 요청 +0.02 m/s이고 <0.15 m/s | 자동 복귀 |
| 과속 후 새 pulse가 5초간 없음 | 무펄스 복구 조건으로 자동 복귀 가능. 물리 정지를 증명하지 않음 |
| 요청속도 0 | BRAKE. 정상 손 해제 ramp는 아래 예외 |

MCU 과속 판정은 pulse-period 원시 속도와 필터 속도 중 큰 크기를 사용한다.
각도와 독립적인 조건이며 Pi가 실시간으로 새 속도를 받는다는 가정에 의존하지 않는다.
다만 6 pulse/rev의 관측 지연 때문에 즉각적인 과속 감지는 불가능하다.

**기존** 양손 dead-man, 통신 watchdog, 하드웨어 fault, 수동 inhibit는 유지했다.
정상 손 해제는 기존 600ms 동안 직전 출력부터 단조 감소시킨다. E-stop 설정은
기존과 같이 비활성이며, 통신/하드웨어 fault는 기존 상태기계의 정지·복구 절차를 따른다.
새 경사 BRAKE 자체는 `ARMED`를 유지하므로 별도 재시작 버튼을 요구하지 않는다.

단락 BRAKE는 기계식 주차 브레이크가 아니다. 영속도에서 경사 위 위치를 유지하거나
전원이 없는 상태에서 제동을 보장하지 않는다. 이 구현에 기계적 HOLD는 없다.

## 5. Hall 5초 설정과 측정의 의미

한 pulse 이동거리는 `2π × 0.115 / 6 ≈ 0.1204 m`다.

| 실제 속도 | pulse 간격 |
|---:|---:|
| 0.08 m/s | 1.51초 |
| 0.048 m/s | 2.51초 |
| 0.04 m/s | 3.01초 |
| 0.032 m/s | 3.76초 |

`HALL_ZERO_TIMEOUT_US=5000000`으로 연장했다. 유효 speed는 pulse period가
20ms 이상·5초 미만이고 마지막 pulse age도 5초 미만인 경우다. 첫 pulse만으로는
속도를 만들지 않는다. 필터는 새 pulse가 들어올 때만 갱신한다.
5초 이후 표시 속도는 0으로 정리하되 `speed_valid=false`로 구분하고 P에는 넣지 않는다.

초기/무효 측정에서는 P=0, FF 요구를 최대 60으로 제한한다. 이미 더 큰 출력으로
주행했다면 정상 하강 slew로 줄어든다. 무펄스 구동 감시는 30 미만 출력도 포함하며
`max(5초, 예상 pulse 간격 × 2.5)`를 쓰되 최대 10초로 제한했다. 따라서 0.032 m/s에서
약 9.4초, 0.08 m/s에서는 최소 5초를 기준으로 스톨을 감시한다.
5초는 속도 age timeout이며 스톨 timeout과 같지 않다.

기존 250ms pulse 무시 구간은 약 4.19 rad/s 이상을 표현하지 못해 5 rad/s 이상
검사와 충돌했다. 이를 20ms로 줄였으며 ADC 30/12 히스테리시스는 유지했다.
빠른 ADC pulse를 실제 입력 검출 경로로 통과시키는 회귀 테스트를 추가했다.
노이즈·자석 폭·배선 EMI에 따른 오검출은 실물 튜닝이 필요하다.

## 6. 진단과 업데이트

`/walker/status`에 `ff_pwm`, `feedback_pwm`, `applied_pwm`, `measured_speed_m_s`,
`speed_age`, `speed_valid`, `new_pulse`, `braking`, `direction_valid`가 있다.
`new_pulse`는 직전 성공 telemetry 전송 이후 pulse가 있었음을 뜻한다.
오른쪽 속도는 왼쪽 복제값이고 `direction_valid=false`다.
상태 메시지가 반복돼도 새로운 물리 측정이 생긴 것은 아니다.

`/diagnostics`에는 정규화 pitch·경사 상태·목표속도·slope FF·MCU PWM·속도 age/valid와
BRAKE 상태를 기록한다. 명시적 경사·과속 BRAKE 처리 시 MCU FF/P 표시값과 applied PWM은 0이다.

프로토콜 **v5 / schema 0x0501 / release 20260906**은 v4와 호환되지 않는다.
COMMAND는 12 bytes, Drive telemetry는 54 bytes다. [상세 형식](../PROTOCOL.md).
**Drive Uno, Terrain Uno, Pi ROS 메시지와 노드를 함께 갱신**해야 한다.

```bash
# Pi에서 저장소를 pdj1로 갱신한 뒤, 구동 서비스를 정지한 상태에서 수행
bash scripts/build.sh
arduino-cli compile --fqbn arduino:avr:uno firmware/safestride_mcu
arduino-cli compile --fqbn arduino:avr:uno firmware/terrain_mcu
# 포트 확인 후 각각 upload. 이 코드 작업에서는 실제 업로드하지 않았다.
arduino-cli upload --fqbn arduino:avr:uno -p /dev/safestride-drive firmware/safestride_mcu
arduino-cli upload --fqbn arduino:avr:uno -p /dev/safestride-terrain firmware/terrain_mcu
SAFESTRIDE_ENABLE_PERCEPTION=false SAFESTRIDE_REQUIRE_SURFACE_CONDITION=false \
  SAFESTRIDE_ENABLE_CRUISE=false bash scripts/run.sh
```

이전 `tools/windows_uno_test.ps1`은 v2 도구여서 이번 프로토콜에 사용하지 않는다.
읽기용 연결 검사는 현재 Python 프로토콜을 쓰는 `tools/serial_probe.py`를 사용한다.
노면 활성화 후속 계획은 [MCU README](../firmware/safestride_mcu/README.md)에 있다.

## 7. 검증과 다음 실물 시험

로컬 검증 결과: Drive/Terrain Uno AVR 컴파일과 C++ 호스트 펌웨어 8종 통과.
Python 62개(bridge/프로토콜 36, 감독 명령 7, 경사 정책 6, 설정 일치 13) 통과.
두 실행 YAML의 파싱 및 노면 비활성·오르막 배율 1.0 확인, Python compileall 통과.
최종 Drive 빌드는 flash 15,230/32,256 bytes, 전역 RAM 988/2,048 bytes,
Terrain은 flash 10,648 bytes, 전역 RAM 1,065 bytes다.
이 검사는 실제 DDS 통신, Pi에서의 ROS 실행, USB 연결, 모터 부하 시험을 대체하지 않는다.

실물 시험은 다음 순서로 기록한다.

1. 바퀴를 든 상태에서 LOW/LOW BRAKE와 방향을 확인한다. 평지 장착 pitch의
   offset·sign, 양손 압력, 5초 Hall age를 함께 확인한다.
2. PWM 30/40/60에서 기동과 회전 유지 조건을 각각 측정하고 `v_nom`과 FF를 보정한다.
3. +5°/-5°/-8°에서 목표속도·slope FF·applied PWM을 기록한다. -10° BRAKE와
   -7° 복구, MPU 분리/복구를 시험한다. 조건 해소 시 자동 재개됨에 유의한다.
4. Hall 3.76초 간격에서 false stop이 없는지, 센서 누락 시 출력이 계속 증가하지
   않는지 확인한다. 손 해제 출력이 증가하지 않아야 한다.
5. 제한된 부하에서 두 모터 합산 전류·드라이버 온도·12V/5V 전압·제동거리·미끄러짐을
   측정한다. 속도/각도/전류 한계와 실제 정지 유지 수단은 이 결과로 결정한다.

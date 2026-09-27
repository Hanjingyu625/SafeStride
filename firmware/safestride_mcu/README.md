# Drive Uno firmware

단일 모터드라이버, 왼쪽 D2 A3141/A3144 디지털 Hall, 왼쪽 A2/오른쪽 A1 압력 dead-man과 CRC serial watchdog을
담당한다. Hall 출력의 HIGH→LOW 전환을 외부 인터럽트로 센다. LOW 유지나 HIGH 복귀는
추가 펄스가 아니며, 부팅 시 LOW여도 회전으로 세지 않는다. 회전당
12 pulse이며 압력 threshold는 좌우 35이다. `HALL_CALIBRATED=true`, `PRESSURE_THRESHOLDS_CALIBRATED=true`,
`MAGNET_BENCH_MODE=false`, `ENABLE_ESTOP=false`가 운영 기본값이다.

오른쪽 Hall 입력은 없다. protocol의 오른쪽 pulse/velocity는 왼쪽 값을 복제한
공통 드라이브 추정치이다. 횡방향 조향이 중요하지 않은 시스템이기 때문에, 한쪽 값을 바탕으로 종방향 속도만 추종한다.

첨부 A3141~A3144 데이터시트의 UA 3핀 소자는 글씨 면을 정면으로 보고 다리를 아래로 하면
왼쪽부터 `VCC(5V)`, `GND`, `OUT(D2)`이다. 모듈 제품이면 소자 다리 순서가 아닌 모듈 단자 표기를 따른다.
기존 A3의 OUT 선을 Drive Uno D2로 옮긴다. D2는 모터 PWM D5, 방향 D6/D8, 압력 A2/A1과 겹치지 않는다.
OUT-5V 사이 4.7kΩ 풀업 저항, VCC-GND 사이 0.1µF 바이패스 커패시터를 센서 가까이에 연결한다.
펌웨어의 내부 풀업도 활성화하지만 긴 배선은 외부 풀업을 사용한다. 기존 아날로그 OUT의 큰 필터 커패시터는 제거한다.
전원/풀업은 5V를 사용하고 OUT에 배터리 12V를 연결하지 않는다.

A314x는 단극성이다. 기존 WSH135처럼 양쪽 자극을 모두 감지하지 않는다.
모든 바퀴 자석을 감지되는 극으로 배치하고, 모터 출력을 끈 상태로 천천히 한 바퀴 돌려
`/wheel/hall`의 왼쪽 누적 펄스가 정확히 12 증가하는지 확인한다. 자석은 각 통과 사이에 LOW→HIGH로 복귀할 간격이 필요하다.
이 코드의 `HALL_CALIBRATED=true`는 기존 설정 보존이며 새 장착의 실측 교정 완료를 의미하지 않는다.
6개만 검출된다면 우선 자석 극성을 고친다. 실제 펄스 수를 변경할 경우 MCU와 Pi의 회전당 펄스 설정을 함께 맞춰야 한다.

20ms 최소 펄스 간격, 5초 측정 유효기간, 스톨·과속 보호, 속도/PWM 정책과 protocol v6는 유지한다.
인터럽트에서 시간만 기록하고 제어 루프에서 원자적으로 복사하므로 5ms 제어 주기보다 짧은 LOW도 검출한다.
ADC 영점 학습은 필요 없다. 이 펌웨어는 이전 아날로그 WSH135 입력과 호환되지 않는다.

```bash
arduino-cli compile --fqbn arduino:avr:uno firmware/safestride_mcu
arduino-cli upload --fqbn arduino:avr:uno -p /dev/safestride-drive \
  firmware/safestride_mcu
```


## pdj1 속도제어 (2026-09-06)

프로토콜 v6로 목표속도·경사 FF·PWM cap·BRAKE/TERRAIN_STOP을 함께 받는다. 평지 기준은 PWM
60, FF bias는 30이며 하한을 강제하지 않는다. Hall 무펄스 대기는 5초다.
DRI0042 참고 제어표의 BRAKE(LOW/LOW)를 유지하고 경사 조건 해소 시 자동 복귀한다.
기존 dead-man·watchdog·하드웨어 fault 처리는 유지한다.
전체 제어식·파라미터·업데이트 순서는 [속도제어 문서](../../docs/SPEED_CONTROL_KO.md)를 본다.

## 다음 수정 계획: 노면인식 연동

현재 학습이 미완료여서 `SAFESTRIDE_ENABLE_PERCEPTION=false`,
`require_surface_condition=false`, `surface_control_enabled=false`를 유지한다.
예전 모델 파일은 보존하지만 감독 노드는 수신한 노면 결과도 제어에 적용하지 않는다.

1. 실제 보행기 카메라의 평지·거친 노면·젖은 노면·자갈 데이터를 수집한다.
   같은 영상의 인접 프레임을 train/validation/test에 나누지 않고 촬영 구간별로 분리한다.
   class별 recall, confusion matrix, 조명·흔들림·카메라 각도별 실패 사례를 기록한다.

2. Pi에서 FPS·추론 지연·메모리·CPU 사용량을 측정한다. confidence 기준과 class
   확정/해제 dwell을 검증하고 모델 hash와 class 순서를 manifest에 고정한다.

3. 먼저 모터 제어와 분리해 `/perception/surface_condition`만 기록한다.
   unknown·낮은 confidence·stale 결과의 처리 정책을 확정한 뒤 제어 활성화를 검토한다.

4. `surface_control_enabled=true`를 두 YAML에 함께 적용할 때 목표속도 배율은
   기존 `combine_speed_scales()`를 사용한다. wet 감속을 uphill FF가 상쇄하지 않도록
   노면별 `drive_pwm_cap`을 별도로 설계하고 같은 `DriveCommand`에 담는다.
   cap은 정상 slew 중에도 즉시 지켜져야 하며 별도 오래된 토픽과 혼합하지 않는다.

5. `require_surface_condition` 및 launch 기본값을 의도적으로 선택한다.
   enabled인데 모델/카메라가 없는 경우, 재연결, 오래된 메시지, 급경사와 wet 동시
   진입, 손 해제, watchdog을 회귀 테스트한다. 학습 완료만으로 자동 활성화하지 않는다.

6. 바퀴를 든 시험 후 제한된 부하 시험에서 실제 속도·전류·미끄러짐·제동거리를
   측정하고 배율/cap을 조정한다. 모델·노면별 결과와 코드 버전을 문서화한다.

7. 학습 정확도가 향상되고 난 뒤, 'SAFESTRIDE_ENABLE_PERCEPTION=True' 로 설정한 뒤 모터와 연동하여 테스트한다.

추가 개선 후보: MPU 자이로 융합/축 보정, Hall 분해능 증가 또는 방향 엔코더,
전류 피드백, 실제 BRAKE 성능 측정. Ki/Kd는 최종 포화·slew까지 고려한
anti-windup과 측정 주기 처리를 구현하기 전까지 0으로 유지한다.

# Drive Uno firmware

단일 모터드라이버, 왼쪽 A3 WSH135 Hall, 왼쪽 A2/오른쪽 A1 압력 dead-man과 CRC serial watchdog을
담당한다. WSH135는 부팅 시 무자계 기준값을 학습하고, 기준값에서 30 ADC 이상
벗어나면 자석 1개를 센 뒤 12 ADC 이내로 돌아와야 다음 pulse를 센다. 회전당
12 pulse이며 압력 threshold는 좌우 80이다. `HALL_CALIBRATED=true`, `PRESSURE_THRESHOLDS_CALIBRATED=true`,
`MAGNET_BENCH_MODE=false`, `ENABLE_ESTOP=false`가 운영 기본값이다.

오른쪽 Hall 입력은 없다. protocol의 오른쪽 pulse/velocity는 왼쪽 값을 복제한
공통 드라이브 추정치다. 실제 회전 방향은 측정할 수 없어 부호는 명령 방향을
따른다.

WSH135 배선은 마킹이 보이는 평평한 면을 정면으로 보고 다리를 아래로 했을 때
왼쪽부터 `VDD(5V)`, `GND`, `OUT(A3)`이다. 출력에는 저항 부하를 달지 않고,
노이즈가 있으면 OUT-GND 사이에 0.01~0.1 uF 커패시터를 센서 가까이에 단다.
전원을 넣을 때는 자석이 센서 앞에 없도록 둔다.

```bash
arduino-cli compile --fqbn arduino:avr:uno firmware/safestride_mcu
arduino-cli upload --fqbn arduino:avr:uno -p /dev/safestride-drive \
  firmware/safestride_mcu
```

E-stop 설치 전에는 별도의 물리 모터 전원 차단 수단을 사용한다.

## pdj1 속도제어 (2026-09-06)

프로토콜 v5로 목표속도·경사 FF·PWM cap·BRAKE를 함께 받는다. 평지 기준은 PWM
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

추가 개선 후보: MPU 자이로 융합/축 보정, Hall 분해능 증가 또는 방향 엔코더,
전류 피드백, 실제 BRAKE 성능 측정. Ki/Kd는 최종 포화·slew까지 고려한
anti-windup과 측정 주기 처리를 구현하기 전까지 0으로 유지한다.

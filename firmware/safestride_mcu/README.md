# SafeStride Drive MCU 펌웨어

Arduino Uno가 단일 SZH-GNP521을 통해 두 모터에 공통 속도 명령을 내리고,
향후 휠 엔코더 피드백과 좌우 압력센서를 감시한다. E-stop은 현재 미구현이다.
직렬 통신은 [프로토콜 v3](../../PROTOCOL.md)를 사용한다.

## 핀맵

| 기능 | Uno 핀 |
|---|---:|
| 엔코더 입력 1/2 예약 | D2 / D3 (interrupt 가능) |
| 단일 드라이버 PWM / INA(IN1) / INB(IN2) | D5 / D6 / D8 |
| 왼쪽/오른쪽 압력센서 | A1 / A2 |
| E-stop | 현재 미구현·미사용, D12 예약 |
| 드라이버 fault(기본 비활성) | D13 |

D2/D3의 실제 역할은 엔코더 선정 후 확정한다. 하나의 quadrature 엔코더 A/B로
쓸지, 좌우 단채널 입력으로 쓸지 아직 정해지지 않았으므로 현재 코드는 핀 모드나
인터럽트 에지를 설정하지 않는다. D4, D7, D9, D10은 비어 있다.

## 현재 제어 조건

`encoder_feedback.cpp`는 하드웨어 중립 인터페이스만 제공하며 현재 항상 invalid
샘플을 반환한다. `ENABLE_ENCODER_FEEDBACK=false`와
`ALLOW_OPEN_LOOP_MOTOR=true`인 현재 설정은 들어 올린 바퀴의 임시 시험을 위한
open-loop 모드다. 이때도 세션, 명시적 ROS enable, 최신 속도 명령, dead-man,
watchdog, fault 조건을 모두 통과해야 PWM이 출력된다.

엔코더 피드백을 켜면 firmware는 다음 조건을 모두 만족하지 않는 한 모터 출력을
차단한다.

1. 구매한 엔코더용 adapter가 초기화되어 capability를 광고한다.
2. `ENCODER_CALIBRATED=true`이고 회전당 카운트, 기어비, 방향이 검증됐다.
3. 매 control cycle의 샘플이 valid다.
4. 정지 상태 dwell, dead-man, session, command watchdog, fault 조건이 정상이다.

실제 운용 전에는 `ALLOW_OPEN_LOOP_MOTOR=false`, ROS의
`require_encoder_feedback=true`로 바꿔 MCU와 Raspberry Pi 양쪽에서 피드백 없는
구동을 차단한다.

## 엔코더 선정 후 구현할 항목

- 전원/논리 전압과 level shifting
- 출력 방식(push-pull/open-collector/line-driver), pull-up 및 edge polarity
- single/dual channel 또는 quadrature decoding 방식
- 축 1회전당 카운트, 감속비, 휠 출력축 환산값
- 정·역방향 부호와 좌우 배선
- 저속 샘플링/속도 필터, overflow 처리, stall/overspeed 임계값

어댑터는 `WheelEncoderSample`의 위치(mrad), 속도(mrad/s), valid를 채우도록
구현한다. 최초 시험은 바퀴를 든 상태에서 수행하고 PID 적분 게인은 방향과
스케일이 검증되기 전까지 0으로 둔다.

## 빌드

```bash
arduino-cli compile --fqbn arduino:avr:uno firmware/safestride_mcu
arduino-cli upload --fqbn arduino:avr:uno -p /dev/safestride-drive \
  firmware/safestride_mcu
```

두 모터를 하나의 드라이버 출력에 연결하는 설계는 코드만으로 안전성을 보장할
수 없다. 모터 합산 정지전류, 드라이버/배터리/퓨즈/배선 정격을 확인하고, E-stop이
구현되기 전에는 별도의 물리 전원 차단 수단을 준비한다.

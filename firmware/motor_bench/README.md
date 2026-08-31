# 간단 단일 드라이버 벤치

기존 `M,<signed PWM>` 명령을 유지하는 최소 시험 스케치이다. 하나의
SZH-GNP521이 두 모터를 함께 구동하며 핀은 PWM=D5, IN1=D6, IN2=D8이다.

```text
M,20
M,-20
M,0
```

안전한 시간 제한·CONFIRM·watchdog 시험에는
`firmware/motor_driver_bench_test`를 우선 사용한다. 운영에서는 이 스케치를
사용하지 말고 홀센서, E-stop, 압력 dead-man과 세션 watchdog이 적용된
`safestride_mcu`를 사용한다.

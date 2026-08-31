# Drive Uno firmware

단일 모터드라이버, 왼쪽 D2 Hall, 왼쪽 A2/오른쪽 A1 압력 dead-man과 CRC serial watchdog을
담당한다. D2는 `INPUT_PULLUP`/`FALLING`, 회전당 6 pulse이며 threshold는 좌우
80이다. `HALL_CALIBRATED=true`, `PRESSURE_THRESHOLDS_CALIBRATED=true`,
`MAGNET_BENCH_MODE=false`, `ENABLE_ESTOP=false`가 운영 기본값이다.

오른쪽 Hall 입력은 없다. protocol의 오른쪽 pulse/velocity는 왼쪽 값을 복제한
공통 드라이브 추정치다. 실제 회전 방향은 측정할 수 없어 부호는 명령 방향을
따른다.

```bash
arduino-cli compile --fqbn arduino:avr:uno firmware/safestride_mcu
arduino-cli upload --fqbn arduino:avr:uno -p /dev/safestride-drive \
  firmware/safestride_mcu
```

E-stop 설치 전에는 별도의 물리 모터 전원 차단 수단을 사용한다.

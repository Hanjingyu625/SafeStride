# Drive Uno firmware

단일 모터드라이버, 왼쪽 A3 WSH135 Hall, 왼쪽 A2/오른쪽 A1 압력 dead-man과 CRC serial watchdog을
담당한다. WSH135는 부팅 시 무자계 기준값을 학습하고, 기준값에서 30 ADC 이상
벗어나면 자석 1개를 센 뒤 12 ADC 이내로 돌아와야 다음 pulse를 센다. 회전당
6 pulse이며 압력 threshold는 좌우 80이다. `HALL_CALIBRATED=true`, `PRESSURE_THRESHOLDS_CALIBRATED=true`,
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

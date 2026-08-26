# 단일 SZH-GNP521 모터 테스트

단일 드라이버에 두 모터를 하나의 부하로 연결하는 현재 구성을 ROS 없이
시험한다. Uno 핀은 PWM=D5, INA(IN1)=D6, INB(IN2)=D8, COM=GND이다. 상태 LED는
사용하지 않고 모든 결과를 115200 baud 터미널에 출력한다.

실물 보드의 `5VO`는 출력이므로 Uno 5 V와 연결하지 않는다. 리비전에 따라
`VCC`/`5V IN`으로 표시된 입력이 있다면 해당 보드 사양대로 5 V를 공급한다.

두 모터는 동일 정격이어야 하며 설계가 병렬 연결을 전제로 한다. 전원 인가
전에 두 모터 합산 정지전류가 드라이버, 배터리, 퓨즈와 배선의 연속 허용치를
넘지 않는지 확인한다. 좌우 장착 방향이 반대라면 한 모터의 두 출력선을
서로 바꿔 같은 명령에서 두 바퀴가 모두 전진하도록 맞춘다.

```powershell
arduino-cli compile --fqbn arduino:avr:uno firmware/motor_driver_bench_test
arduino-cli upload --fqbn arduino:avr:uno -p COM3 firmware/motor_driver_bench_test
```

두 바퀴를 모두 들어 올리고 시리얼 모니터를 115200 baud/Newline으로 연다.

```text
STATUS
RUN 20 500 CONFIRM
RUN 40 500 CONFIRM
RUN -20 500 CONFIRM
STOP
```

PWM은 `-100..100`, 시간은 `50..3000 ms`로 제한되며 시간이 끝나면 자동
정지한다. 잘못된 명령, 긴 명령 또는 watchdog reset도 즉시 PWM을 0으로
만든다. 테스트 후 운영 펌웨어를 다시 업로드해야 한다.

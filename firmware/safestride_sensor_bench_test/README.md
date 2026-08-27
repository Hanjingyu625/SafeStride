# Drive sensor bench

모터 전원을 분리한 상태에서 왼쪽 휠 D2 홀센서와 A0/A1 압력센서를 확인한다.
홀 입력은 `INPUT_PULLUP`과 `FALLING` edge를 사용하며 왼쪽 휠의 자석은 6개다.
압력 판정 기준은 좌우 모두 ADC 25다.

115200 baud 시리얼 출력에서 휠 1회전당 pulse가 6 증가하는지, 휠 속도에 따라
RPM이 변하는지, 양손을 누를 때 `deadman=1`이 되는지 확인한다. 스케치는
D5/D6/D8 모터 출력은 항상 0/LOW/LOW로 유지한다.

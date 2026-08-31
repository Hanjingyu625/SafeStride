# Ubuntu PC에서 Pi 연결이 간헐적으로 끊길 때

현재 전원 경로는 12 V 배터리에서 모터드라이버로 직접 공급하고, XL4015에서
5 V로 강하해 Pi를 공급한 뒤 Pi USB가 Arduino Uno 두 대를 공급하는 구조다.

가장 먼저 확인할 원인은 **Pi 5 V 순간 강하**다. FND가 계속 12 V를 표시하는 것은
배터리 입력이 유지된다는 뜻일 뿐, 모터 기동이나 USB 부하 변화 때 XL4015 출력과
Pi 5 V 핀이 순간적으로 내려가지 않는다는 증거는 아니다. 다음 순서로 구분한다.

1. 모터 전원을 분리한 상태와 연결한 상태에서 각각 SSH 끊김을 재현한다.
2. Pi의 5 V-GND를 Pi 단자에서 측정한다. 가능하면 오실로스코프 또는 최소값
   기록 기능을 사용하고, XL4015 출력선은 짧고 굵게 배선한다.
3. `scripts/diagnose_pi_connection.sh` 결과의 `get_throttled`와 USB/Wi-Fi 로그를
   저장한다.
4. Wi-Fi power save가 `on`이면 시험 중만 `sudo iw dev wlan0 set power_save off`
   로 비교한다.
5. `ping PI_IP`도 함께 끊기면 전원·Wi-Fi·DHCP 문제이고, ping은 유지되는데 SSH만
   끊기면 sshd 로그와 PC의 keepalive 설정을 확인한다.

권장 SSH client 설정은 `ServerAliveInterval 10`, `ServerAliveCountMax 3`이다.
이는 실제 전원/무선 단절을 고치는 설정이 아니라 유휴 NAT timeout을 구분하고
끊김을 빨리 감지하기 위한 보조 설정이다.

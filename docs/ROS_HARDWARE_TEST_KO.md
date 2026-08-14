# Raspberry Pi + Arduino 2대 ROS 통합 테스트

이 문서는 다음 구성을 한 번에 검사하는 절차다.

- Raspberry Pi 4 / Ubuntu 24.04 / ROS 2 Jazzy
- SafeStride Drive Uno: 좌우 홀센서, 좌우 압력센서, 단일 SZH-GNP521
- Terrain Uno: TOF-10120
- 두 모터: 하나의 SZH-GNP521 출력에 연결되어 같은 명령으로 동작

현재 운영 ROS에서 확인 가능한 센서는 `/wheel/hall`, `/handle/pressure`,
`/terrain/tof`이다. MPU-9250/AK8963과 BNO055는 아직 운영 펌웨어와 프로토콜에
구현되지 않아 이 절차의 검사 대상이 아니다.

## 1. 전원 인가 전 확인

모터 배터리를 분리한 상태에서 시작한다. 사람을 보행기에 태우거나 손으로
바퀴를 붙잡은 상태에서는 구동 시험을 하지 않는다.

### Drive Uno 핀맵

| Uno | 연결 |
|---:|---|
| D2 | 왼쪽 홀센서 출력 |
| D3 | 오른쪽 홀센서 출력 |
| D5 | 단일 드라이버 PWM |
| D6 | 단일 드라이버 INA(코드의 IN1) |
| D8 | 단일 드라이버 INB(코드의 IN2) |
| A0/A1 | 좌우 압력센서 |
| A2 | 예약(E-stop 미구현, 현재 미사용) |
| D13 | 드라이버 fault(기본 비활성) |
| GND | 센서 GND와 드라이버 COM 공통 |

판매처 사양상 드라이버 제어부는 5 V이지만 제품 리비전별 표기가 달라질 수 있다.
실물 단자가 `5VO`처럼 출력으로 표시되어 있으면 Uno 5 V와 연결하지 않고,
`VCC`/`5V IN`처럼 입력으로 표시된 경우에만 규정된 5 V를 공급한다. 두 모터는
동일 전압 정격의 병렬 부하로 연결하는 설계이며, 같은 극성에서 두 바퀴가 모두
전진해야 한다. 반대로 도는 모터가 있으면 전원을 끈 뒤 그 모터의 OUT1/OUT2
선만 바꾼다.

배터리 연결 전에 두 모터의 **합산 정지전류**가 SZH-GNP521, 배터리, 퓨즈,
커넥터와 배선의 허용치를 넘지 않는지 해당 부품 자료로 확인한다. 현재 E-stop은
미구현이므로 시험 중 즉시 사용할 수 있는 별도의 물리 전원 차단 수단을 둔다.
[판매처 표기](https://www.devicemart.co.kr/goods/view?no=1385282)의 최대 전류는
15 A이지만 이를 연속 허용전류로 간주해서는 안 되며, 실물 리비전의 방열 조건도
확인해야 한다.

### Terrain Uno 핀맵

| Uno | 연결 |
|---:|---|
| A4/SDA | TOF/MPU/BNO SDA |
| A5/SCL | TOF/MPU/BNO SCL |
| GND | 모든 센서 공통 GND |
| 5V/3.3V | 각 브레이크아웃 사양에 맞는 전원 |

상태 LED는 두 보드 모두 사용하지 않는다. 결과는 터미널 또는 ROS 토픽에서만
확인한다.

## 2. 홀센서 보정

먼저 3~6절에 따라 `HALL_CALIBRATED=false`인 운영 펌웨어와 ROS bridge를
실행한다. `/wheel/hall`의 시작값을 기록하고 각 휠을 정확히 10회 회전한다.
좌우 `pulses` 종료값과 시작값 차이의 절댓값을 각각 10으로 나눈 값이 휠
1회전당 펄스 수다.

같은 값을 다음 세 곳에 반영한다.

1. `firmware/safestride_mcu/config.h`
   - `HALL_PULSES_PER_WHEEL_REV`
   - 검증 완료 후 `HALL_CALIBRATED=true`
2. `config/raspberry_pi.yaml`
   - `base.hall_pulses_per_revolution`
3. `src/safestride_bringup/config/safestride.yaml`
   - `base.hall_pulses_per_revolution`

펄스 수를 알 수 없는 기본 상태에서는 `HALL_CALIBRATED=false`이며 MCU와 ROS
브리지가 모두 모터 enable을 거부한다.

## 3. 두 Uno 운영 펌웨어 업로드

프로토콜 v2 변경이 포함되어 있으므로 **두 Uno를 모두 다시 플래시**해야 한다.
한 번에 보드 하나만 연결하면 포트가 뒤바뀌는 실수를 줄일 수 있다.

```bash
arduino-cli board list
arduino-cli compile --fqbn arduino:avr:uno firmware/safestride_mcu
arduino-cli upload --fqbn arduino:avr:uno -p /dev/ttyACM0 \
  firmware/safestride_mcu

arduino-cli compile --fqbn arduino:avr:uno firmware/terrain_mcu
arduino-cli upload --fqbn arduino:avr:uno -p /dev/ttyACM1 \
  firmware/terrain_mcu
```

운영 펌웨어는 같은 포트에 binary 프레임을 보내므로 Arduino 시리얼 모니터를
열어 두지 않는다. ROS bridge와 시리얼 모니터는 동시에 같은 포트를 사용할 수
없다.

## 4. Raspberry Pi 장치명 고정

두 보드를 모두 연결하고 고유 경로를 확인한다.

```bash
ls -l /dev/serial/by-id/
udevadm info -a -n /dev/ttyACM0 | less
udevadm info -a -n /dev/ttyACM1 | less
```

`deploy/udev/99-safestride.rules.example`의 VID/PID/serial을 실제 값으로 바꾼
뒤 설치한다. 두 보드가 같은 serial을 갖거나 serial이 없다면 이 규칙을 그대로
사용하지 말고 USB 물리 경로 기준 규칙을 만들어야 한다.

```bash
sudo cp deploy/udev/99-safestride.rules.example \
  /etc/udev/rules.d/99-safestride.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
ls -l /dev/safestride-drive /dev/safestride-terrain
```

현재 `config/raspberry_pi.yaml`은 위 두 고정 경로를 사용한다. 사용자를
`dialout` 그룹에 추가한 뒤에는 다시 로그인한다.

```bash
sudo usermod -aG dialout "$USER"
```

## 5. Raspberry Pi 빌드와 소프트웨어 검사

```bash
cd ~/SafeStride
bash scripts/install_ubuntu_24_04.sh   # 최초 1회
# 다시 로그인한 뒤
bash scripts/build.sh
bash scripts/test.sh
```

`scripts/test.sh`는 ROS 패키지 테스트, 프로토콜 테스트, 펌웨어 host 테스트와
운영 핀맵 무결성을 검사한다. 실패가 하나라도 있으면 모터 전원을
연결하지 않는다.

유선 LAN/SSH 설정은 [ETHERNET.md](ETHERNET.md)를 따른다. 직접 연결의 기본
주소는 PC `10.42.0.1`, Pi `10.42.0.2`이며 ROS domain은 `42`다.

## 6. 모터 전원 없이 ROS 센서 확인

Pi의 첫 번째 터미널에서 실행한다.

```bash
cd ~/SafeStride
bash scripts/run.sh
```

두 번째 터미널에서 환경을 불러온다.

```bash
source /opt/ros/jazzy/setup.bash
source ~/SafeStride/install/setup.bash
export ROS_DOMAIN_ID=42
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
ros2 node list
ros2 topic list | sort
```

다음 토픽을 하나씩 확인한다.

```bash
ros2 topic echo /walker/status
ros2 topic echo /wheel/hall
ros2 topic echo /handle/pressure
ros2 topic echo /terrain/tof
ros2 topic echo /terrain/status
ros2 topic echo /diagnostics
```

| 조작 | 기대 결과 |
|---|---|
| 좌우 휠을 손으로 회전 | `/wheel/hall`의 해당 pulses 증가 |
| 회전 속도 증가 | 해당 `velocity_rad_s` 절댓값 증가 |
| 양쪽 손잡이를 누름 | `/walker/status.deadman=true` |
| 한쪽 손을 놓음 | `deadman=false` |
| E-stop 상태 | 현재 미구현이므로 `/walker/status.estop=false` 유지 |
| TOF 앞 물체 이동 | `/terrain/tof.range` 변화 |

`/wheel/hall.calibrated=false`, stale telemetry, fault bit 또는
dead-man=false 중 하나라도 있으면 모터 시험으로 넘어가지 않는다.

## 7. 바퀴를 든 상태에서 ROS 모터 시험

1. 두 바퀴가 바닥과 완전히 떨어졌는지 확인한다.
2. 퓨즈와 별도의 물리 모터전원 차단 수단을 즉시 사용할 수 있게 둔다.
3. 모터 배터리를 연결한다.
4. 새 터미널에서 직진 명령을 연속 발행한다. 단일 드라이버이므로
   `angular.z`는 반드시 0이어야 한다.

```bash
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/TwistStamped \
  "{header: {frame_id: base_link}, twist: {linear: {x: 0.15}, angular: {z: 0.0}}}"
```

5. 명령 발행이 유지되는 동안 다른 터미널에서 한 번만 enable한다.

```bash
ros2 service call /walker/set_enabled std_srvs/srv/SetBool "{data: true}"
```

6. 두 모터가 같은 진행방향으로 동작하고 `/wheel/hall`의 양쪽 속도가 함께
   증가하는지 확인한다. 한쪽 펄스가 없으면 약 1.5초 후 Hall fault가 latch되고
   PWM이 0이 되어야 한다.
7. 즉시 정지할 때는 다음 명령을 보내고 속도 발행 터미널도 `Ctrl+C`로 끝낸다.

```bash
ros2 service call /walker/set_enabled std_srvs/srv/SetBool "{data: false}"
```

8. dead-man 한쪽 해제, `/cmd_vel` 발행 중단, Drive Uno USB 분리를
   각각 시험해 모든 경우에 출력이 정지하는지 확인한다. Hall fault는 MCU
   reset까지 유지되므로 원인을 해결한 뒤 Drive Uno를 재부팅한다.

회전 명령(`angular.z != 0`)은 safety supervisor와 ROS bridge가 거부하며 enable 요청도
해제한다. 두 모터를 서로 다른 속도로 구동하거나 제자리 회전하는 기능은 현재
단일 드라이버 하드웨어에서는 지원하지 않는다.

## 8. 정상 판정 체크리스트

- [ ] `/dev/safestride-drive`, `/dev/safestride-terrain`이 올바른 보드를 가리킨다.
- [ ] 두 bridge가 protocol v2 session을 시작한다.
- [ ] `/wheel/hall.calibrated=true`이고 좌우 펄스·속도가 모두 변한다.
- [ ] 압력센서와 TOF 토픽이 실제 조작에 반응하고 E-stop은 false로 유지된다.
- [ ] `/diagnostics`에 serial timeout, CRC, frame, Hall fault가 없다.
- [ ] enable 전에는 배터리가 연결되어도 두 모터가 움직이지 않는다.
- [ ] enable 후 두 모터가 같은 방향으로 움직인다.
- [ ] dead-man, 명령 timeout, USB 분리 시 모두 즉시 정지한다.
- [ ] 시험 종료 후 `/walker/set_enabled false`와 물리 전원 분리를 완료했다.

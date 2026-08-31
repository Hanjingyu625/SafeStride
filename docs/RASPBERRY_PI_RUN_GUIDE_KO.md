# Raspberry Pi 연결·실행·ROS 2 토픽 확인 가이드

이 문서는 Ubuntu PC에서 Raspberry Pi 4에 접속하고 SafeStride `pdj` 브랜치를
빌드한 뒤, 모터를 활성화하지 않은 상태에서 센서 토픽을 확인하는 절차를 설명한다.

> SafeStride는 인증된 안전 제어기가 아니다. E-stop이 없으므로 최초 시험은
> 모터드라이버의 12 V 전원을 분리하고 바퀴를 든 상태에서 진행한다. 물리적인
> 모터 전원 차단 수단을 손이 닿는 곳에 둔다.

## 1. 전체 연결 순서

```text
Ubuntu PC -- SSH/Ethernet or Wi-Fi --> Raspberry Pi 4
                                         |
                                         +-- USB --> Drive Uno
                                         |           D2 left Hall, left A2/right A1 pressure
                                         |
                                         +-- USB --> Terrain Uno
                                         |           TOF10120, MPU6050
                                         |
                                         +-- serial --> BE-220 GPS
```

초기 확인은 다음 순서로 진행한다.

1. PC에서 Pi까지 `ping`과 SSH를 확인한다.
2. Pi에서 두 Uno의 고정 장치 이름을 확인한다.
3. ROS 워크스페이스를 빌드하고 테스트한다.
4. cruise와 모터를 비활성화한 상태로 ROS 노드를 실행한다.
5. 센서와 진단 토픽을 확인한다.
6. HIL smoke test를 통과한 뒤에만 들린 바퀴로 제한 시간 모터 시험을 한다.

## 2. PC에서 Raspberry Pi 접속

### 같은 공유기 또는 스위치를 사용하는 경우

Pi에서 주소를 확인한다.

```bash
ip -brief -4 address
hostname
```

Ubuntu PC에서 접속한다. 아래 `ubuntu`는 실제 Pi 사용자 이름으로 바꾼다.

```bash
ping raspberrypi.local
ssh ubuntu@raspberrypi.local
```

mDNS가 되지 않으면 Pi에 표시된 IPv4 주소를 사용한다.

```bash
ssh ubuntu@192.168.0.50
```

### PC와 Pi를 Ethernet 케이블로 직접 연결하는 경우

먼저 Pi를 모니터·키보드 또는 기존 Wi-Fi로 접속한 상태에서 실행한다.

```bash
cd ~/SafeStride
sudo bash scripts/configure_pi_ethernet.sh direct eth0
```

Pi 주소는 `10.42.0.2/24`가 된다. Ubuntu PC에서 Ethernet 인터페이스 이름을
확인하고, 처음 한 번만 전용 연결을 만든다.

```bash
ip -brief link
PC_ETH_IF=enp3s0  # 실제 Ubuntu PC Ethernet 인터페이스로 변경
sudo nmcli connection add \
  type ethernet \
  ifname "${PC_ETH_IF}" \
  con-name safestride-direct \
  ipv4.method manual \
  ipv4.addresses 10.42.0.1/24 \
  ipv6.method disabled
sudo nmcli connection up safestride-direct
ping 10.42.0.2
ssh ubuntu@10.42.0.2
```

이미 `safestride-direct` 연결이 있으면 새로 만들지 말고 다음 명령만 사용한다.

```bash
sudo nmcli connection up safestride-direct
```

## 3. Pi에 `pdj` 브랜치 설치

처음 clone하는 경우:

```bash
git clone --branch pdj --single-branch \
  https://github.com/Hanjingyu625/SafeStride.git ~/SafeStride
cd ~/SafeStride
git status --short --branch
```

이미 저장소가 있는 경우에는 로컬 변경을 먼저 보존한 뒤 다음을 실행한다.

```bash
cd ~/SafeStride
git fetch origin
git switch pdj
git pull --ff-only origin pdj
```

운영체제는 Raspberry Pi 4 arm64 Ubuntu Server 24.04를 기준으로 한다.

```bash
cd ~/SafeStride
bash scripts/install_ubuntu_24_04.sh
```

설치 스크립트가 사용자를 `dialout`, `video` 그룹에 추가하므로, 완료 후 SSH에서
나갔다가 다시 접속한다.

```bash
exit
```

Ubuntu PC에서 다시 접속한다.

```bash
ssh ubuntu@raspberrypi.local
cd ~/SafeStride
bash scripts/build.sh
bash scripts/test.sh
```

## 4. 두 Arduino Uno, GPS와 고정 장치 이름 확인

모터드라이버의 12 V 전원을 분리한 상태에서 두 Uno와 BE-220 serial 장치를
Raspberry Pi에 연결한다.

```bash
ls -l /dev/ttyACM* /dev/ttyUSB* /dev/serial/by-id/ 2>/dev/null
for port in /dev/ttyACM* /dev/ttyUSB*; do
  [[ -e "${port}" ]] || continue
  echo "== ${port} =="
  udevadm info --query=property --name="${port}" |
    grep -E 'ID_VENDOR_ID|ID_MODEL_ID|ID_SERIAL_SHORT'
done
```

출력된 `ID_SERIAL_SHORT`가
`deploy/udev/99-safestride.rules`의 Drive/Terrain 값과 일치하는지 확인한다.
GPS는 USB-UART 장치가 아니므로 udev 별칭을 만들지 않고 Pi GPIO UART
별칭인 `/dev/serial0`을 사용한다.

```bash
sudo install -m 0644 deploy/udev/99-safestride.rules \
  /etc/udev/rules.d/99-safestride.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
```

장치를 다시 연결하고 두 Uno 링크와 Pi GPIO UART가 준비됐는지 확인한다.

```bash
ls -l /dev/safestride-drive /dev/safestride-terrain /dev/serial0
test -r /dev/safestride-drive && test -w /dev/safestride-drive
test -r /dev/safestride-terrain && test -w /dev/safestride-terrain
test -r /dev/serial0 && test -w /dev/serial0
```

펌웨어 protocol v4가 두 Uno에 모두 올라가 있어야 한다. `arduino-cli`를 사용하는
경우 각각 컴파일·업로드한다. 현재 MCU 펌웨어에는 외부 Arduino 라이브러리가
필요하지 않는다.

```bash
arduino-cli compile --fqbn arduino:avr:uno firmware/safestride_mcu
arduino-cli compile --fqbn arduino:avr:uno firmware/terrain_mcu

arduino-cli upload \
  --port /dev/safestride-drive \
  --fqbn arduino:avr:uno \
  firmware/safestride_mcu
arduino-cli upload \
  --port /dev/safestride-terrain \
  --fqbn arduino:avr:uno \
  firmware/terrain_mcu
```

업로드 중에는 ROS bridge나 Arduino Serial Monitor가 같은 포트를 열고 있으면 안 된다.

## 5. 모터 없이 ROS 노드 실행

서비스가 이미 실행 중이면 먼저 중지한다. 동일한 serial bridge를 두 번 실행하면
안 된다.

```bash
sudo systemctl stop safestride.service 2>/dev/null || true
```

첫 번째 SSH 터미널에서 cruise와 perception을 끄고 실행한다.

```bash
cd ~/SafeStride
SAFESTRIDE_ENABLE_CRUISE=false \
SAFESTRIDE_ENABLE_PERCEPTION=false \
bash scripts/run.sh
```

`run.sh`는 `/dev/safestride-drive`, `/dev/safestride-terrain`, `/dev/serial0`의
존재를 확인하고 두 Uno의 serial 역할도 검증한다. 이
실행은 `/walker/set_enabled`를 자동 호출하지 않으므로 Drive MCU는 disarmed
상태여야 한다.

## 6. 두 번째 SSH 터미널에서 topic 확인

새 SSH 터미널을 열고 ROS 환경을 설정한다.

```bash
ssh ubuntu@raspberrypi.local
cd ~/SafeStride
source /opt/ros/jazzy/setup.bash
source install/setup.bash
unset ROS_LOCALHOST_ONLY
export ROS_DOMAIN_ID=42
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
```

노드와 토픽 목록을 확인한다.

```bash
ros2 node list
ros2 topic list -t
ros2 service list -t
```

필수 상태를 한 번씩 확인한다.

```bash
ros2 topic echo /walker/status --once
ros2 topic echo /wheel/hall --once
ros2 topic echo /handle/pressure --once
ros2 topic echo /terrain/status --once
ros2 topic echo /terrain/tof --once
ros2 topic echo /terrain/imu --once
ros2 topic echo /gps/fix --once
ros2 topic echo /gps/course --once
ros2 topic echo /crosswalk/status --once
ros2 topic echo /diagnostics --once
```

주기를 확인할 때는 다음처럼 사용하고 `Ctrl+C`로 종료한다.

```bash
ros2 topic hz /walker/status
ros2 topic hz /terrain/status
ros2 topic hz /terrain/imu
```

정상적인 초기 상태의 핵심 확인점은 다음과 같다.

- `/walker/status`: `link_ok: true`, `armed: false`, `fault_bits: 0`
- `/handle/pressure`: 양쪽 ADC 값과 dead-man 판정이 손 입력에 따라 변함
- `/wheel/hall`: 왼쪽 바퀴를 손으로 한 바퀴 돌리면 pulse가 6 증가함
- `/terrain/status`: 초기 약 10샘플 뒤 `tof_valid: true`
- TOF 정상 기준면: `tof_alert: 0`, `terrain_hazard: false`
- 물체를 가까이 유지: raised 후보 후 `tof_alert: 3`
- 바닥을 멀리 이동: drop 후보 후 `tof_alert: 4`
- MPU6050 미연결은 진단 WARN이지만 TOF 시험 자체는 가능함
- GPS 노드가 `/dev/serial0`을 직접 열며, 유효한 NMEA no-fix 문장은
  `/gps/fix`의 NO_FIX 상태로 발행됨
- 지도·API 미설정 횡단보도 노드는 readiness WARN만 발행하며 모터 명령을 내지 않음

`require_range_sensors=true`이므로 Terrain TOF가 없거나 무효이면 모터 활성은
차단되는 것이 정상이다.

## 7. 자동 HIL smoke test

앞에서 실행한 `run.sh`를 `Ctrl+C`로 종료하고, 모터드라이버 12 V는 계속 분리한다.

```bash
cd ~/SafeStride
bash scripts/hil_smoke_test.sh
```

이 스크립트는 전체 ROS stack을 임시로 실행해 필수 토픽을 확인하고 마지막에
`/walker/set_enabled false`를 호출한다. 모터는 활성화하지 않는다.

## 8. 들린 바퀴로 제한 시간 모터 시험

아래 단계는 센서 토픽과 HIL이 모두 정상일 때만 진행한다.

- 바퀴가 지면과 완전히 떨어져 있어야 한다.
- 양손 압력센서를 계속 잡아야 한다.
- TOF가 정상 기준면을 보고 있어야 한다.
- 물리 모터 전원 차단 수단을 즉시 조작할 수 있어야 한다.

첫 번째 터미널에서 cruise를 끈 stack을 다시 실행한다.

```bash
cd ~/SafeStride
SAFESTRIDE_ENABLE_CRUISE=false bash scripts/run.sh
```

두 번째 터미널에서 5초 제한 시험을 실행한다.

```bash
cd ~/SafeStride
bash scripts/test_drive_pi.sh --enable-motor 5
```

이 스크립트는 `/cmd_vel`을 먼저 발행하고 안전 감독 출력이 양수인지 확인한 뒤에만
Drive를 활성화한다. 종료·오류·`Ctrl+C` 시 `/walker/set_enabled false`를 호출한다.
왼쪽 Hall pulse가 없거나 TOF hazard·dead-man 해제가 발생하면 정지해야 한다.

수동으로 비활성화해야 할 때:

```bash
ros2 service call /walker/set_enabled \
  std_srvs/srv/SetBool "{data: false}"
```

센서 확인만 끝낸 경우 `true` 요청을 보낼 필요가 없다.

## 9. 부팅 시 systemd 서비스로 실행

수동 시험이 끝난 뒤에만 서비스를 설치한다.

```bash
cd ~/SafeStride
bash scripts/install_service.sh
sudoedit /etc/safestride/safestride.env
```

최초 운영 시험에서는 다음 설정을 권장한다.

```text
SAFESTRIDE_ENABLE_CRUISE=false
SAFESTRIDE_ENABLE_PERCEPTION=false
SAFESTRIDE_ENABLE_GPS=true
SAFESTRIDE_ENABLE_CROSSWALK=true
SAFESTRIDE_ENABLE_FOXGLOVE=false
```

서비스를 시작하고 상태·로그를 확인한다.

```bash
sudo systemctl enable --now safestride.service
systemctl status safestride.service --no-pager
journalctl -u safestride.service -f
```

서비스를 중지할 때:

```bash
ros2 service call /walker/set_enabled \
  std_srvs/srv/SetBool "{data: false}" || true
sudo systemctl stop safestride.service
```

## 10. Ubuntu PC에서 직접 ROS topic 보기

가장 단순한 방법은 SSH 터미널 안에서 `ros2 topic echo`를 실행하는 것이다.
Ubuntu PC에도 ROS 2 Jazzy가 설치되어 있고 PC와 Pi가 같은 신뢰된 subnet에 있다면
PC 터미널에서도 다음 환경을 맞춰 직접 조회할 수 있다.

```bash
source /opt/ros/jazzy/setup.bash
unset ROS_LOCALHOST_ONLY
export ROS_DOMAIN_ID=42
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
ros2 daemon stop
ros2 node list
ros2 topic list -t
ros2 topic echo /terrain/status --once
```

SSH는 되지만 PC의 `ros2 node list`가 비어 있으면 PC/Pi의 domain ID, multicast,
방화벽 및 `ROS_LOCALHOST_ONLY`를 확인한다.

## 11. 종료 및 문제 확인

시험 종료 순서는 다음과 같다.

1. `/walker/set_enabled false` 호출
2. `run.sh`를 `Ctrl+C`로 종료하거나 systemd 서비스 중지
3. 모터드라이버 12 V 전원 차단
4. `/walker/status`와 로그의 fault 원인 기록

연결이 간헐적으로 끊기면 다음을 실행한다.

```bash
cd ~/SafeStride
bash scripts/diagnose_pi_connection.sh
journalctl -u safestride.service -n 200 --no-pager
```

FND의 12 V 표시가 유지되어도 XL4015 이후 Pi 단자의 5 V 순간 강하는 별도로
발생할 수 있다. 자세한 점검 순서는 [Pi 연결 진단](PI_CONNECTION_DIAGNOSIS.md)을
참고한다.

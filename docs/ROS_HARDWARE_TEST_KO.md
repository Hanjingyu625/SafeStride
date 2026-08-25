# Raspberry Pi + Arduino 2대 ROS 통합 테스트

현재 운영 구성은 다음과 같다.

- Drive Uno: 공통 모터 드라이버, 왼쪽 바퀴 홀센서 1개, 좌우 압력센서
- Terrain Uno: TOF-10120, BE-220 GPS
- Raspberry Pi: ROS 2 Jazzy, 카메라 노면 분류, 안전감독기
- 기본 속도 요청: 0.08 m/s, 시작 시 모터는 항상 disarmed

현재 Drive 시험 설정도 `REQUIRE_DEADMAN=true`다. 양쪽 압력센서를 모두 가볍게
눌러야 `/walker/status.deadman=true`가 되고 모터를 활성화할 수 있다. 벤치용
임계값은 40 ADC로 완화되어 있지만 실제 손잡이 보정값은 아니므로
`/handle/pressure`의 raw/filtered 값을 기록한 뒤 실사용 임계값으로 교체해야 한다.

## 1. 배선과 전원

첫 시험은 바퀴를 지면에서 들고, 모터 배터리를 분리한 상태로 시작한다. 현재
E-stop 입력은 미구현이므로 별도의 물리 전원 차단 수단을 손 닿는 곳에 둔다.

### Drive Uno

| Uno | 연결 |
|---:|---|
| D2 | 왼쪽 홀센서 출력 |
| D3 | 미사용 |
| D5 | 모터 드라이버 PWM |
| D6 | 모터 드라이버 IN1 |
| D8 | 모터 드라이버 IN2 |
| A1/A2 | 좌우 압력센서 |
| GND | 센서와 드라이버 공통 GND |

홀센서는 모터 시작 스위치가 아니다. 왼쪽 바퀴 회전 펄스로 공통 구동계 속도를
계산한다. 자석을 통과시키면 `/wheel/hall.left_pulses`가 증가하며 오른쪽 값은
공통 추정치로 복제된다. 메시지의 `left_sensor_present=true`,
`right_sensor_present=false`로 실제 장착 여부를 확인한다.

### Terrain Uno와 BE-220

| Terrain Uno | 연결 |
|---:|---|
| A4/A5 | TOF-10120 SDA/SCL |
| D8(RX) | BE-220 TX |
| D9(TX) | BE-220 RX, 설정할 때만 선택적으로 연결 |
| GND | TOF·GPS 공통 GND |

BE-220은 Uno에 연결하기 전에 USB-TTL 어댑터와 제조사 설정 도구로
`9600 baud`에 맞춘다. 정상 운용에는 GPS TX→D8만 있으면 되고 D9는 없어도
된다. GPS 안테나는 하늘이 보이는 야외에 두어야 fix와 속도가 유효해진다.

## 2. Raspberry Pi 코드와 라이브러리

```bash
cd ~/SafeStride
git pull
bash scripts/install_arduino_libraries.sh   # 최초 1회
bash scripts/build.sh
bash scripts/test.sh
```

`AltSoftSerial`, `TinyGPSPlus`, ROS 패키지와 두 펌웨어 테스트가 모두 성공해야
다음 단계로 진행한다.

## 3. 두 Uno 업로드

고정 장치명이 올바른 보드를 가리키는지 먼저 확인한다.

```bash
ls -l /dev/safestride-drive /dev/safestride-terrain
ls -l /dev/serial/by-id/
```

그 다음 두 펌웨어를 모두 컴파일하고 업로드한다.

```bash
cd ~/SafeStride
arduino-cli compile --fqbn arduino:avr:uno firmware/safestride_mcu
arduino-cli upload --fqbn arduino:avr:uno \
  -p /dev/safestride-drive firmware/safestride_mcu

arduino-cli compile --fqbn arduino:avr:uno firmware/terrain_mcu
arduino-cli upload --fqbn arduino:avr:uno \
  -p /dev/safestride-terrain firmware/terrain_mcu
```

업로드 중에는 `scripts/run.sh`과 Arduino 시리얼 모니터를 모두 종료한다.

## 4. 모터 전원 없이 통신 확인

터미널 1에서 카메라 없이 기본 통신부터 확인한다.

```bash
cd ~/SafeStride
SAFESTRIDE_ENABLE_PERCEPTION=false bash scripts/run.sh
```

터미널 2에서 ROS 환경을 불러온다.

```bash
source /opt/ros/jazzy/setup.bash
source ~/SafeStride/install/setup.bash
export ROS_DOMAIN_ID=42
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
```

각 명령은 한 번씩 따로 실행한다.

```bash
ros2 topic echo /walker/status --once
ros2 topic echo /handle/pressure --once
ros2 topic echo /wheel/hall --once
ros2 topic echo /terrain/tof --once
ros2 topic echo /gps/fix --once
ros2 topic echo /gps/speed --once
ros2 topic echo /cmd_vel_safe --once
```

정상 기준은 다음과 같다.

- `/walker/status`: `link_ok: true`, `fault_bits: 0`, telemetry age가 작음
- `/handle/pressure`: raw는 즉시 변하고 filtered는 완만하게 따라가며, 양쪽을
  가볍게 누르면 `left_present/right_present=true`
- `/wheel/hall`: 왼쪽 자석 또는 휠 회전 때 pulse가 증가하고 sensor-present가
  `true/false`로 표시됨
- `/terrain/tof`: 물체 거리를 바꾸면 `range`가 변함
- `/gps/fix`: 실내/미수신은 `status: -1`과 NaN, 야외 fix는 유효 위·경도
- `/gps/speed`: 미수신은 NaN, fix 후 정지는 약 0, 이동하면 m/s 값 증가
- `/cmd_vel_safe`: 약 0.08 m/s까지 가감속 제한에 따라 상승

## 5. 바퀴를 든 모터 시험

1. 바퀴를 지면에서 들고 회전체 주변을 비운다.
2. 물리 전원 차단 수단을 준비한 뒤 모터 배터리를 연결한다.
3. 터미널 1과 기본 정속 명령이 계속 실행 중인지 확인한다.
4. 양쪽 압력센서를 가볍게 누른 상태에서 터미널 2로 한 번만 활성화한다.

```bash
ros2 service call /walker/set_enabled std_srvs/srv/SetBool "{data: true}"
```

`success=True`면 모터가 기본 0.08 m/s 목표로 계속 돌아야 한다. 홀센서 펄스와
`velocity_rad_s`가 회전에 따라 갱신되어야 한다. 4초 동안 필요한 홀 피드백이
없거나 명령·USB 연결이 끊기면 firmware가 출력을 정지한다.

시험을 끝낼 때는 먼저 비활성화한 뒤 배터리를 분리한다.

```bash
ros2 service call /walker/set_enabled std_srvs/srv/SetBool "{data: false}"
```

## 6. 카메라 노면별 속도 시험

터미널 1을 `Ctrl+C`로 종료하고 기본 실행을 다시 시작한다.

```bash
cd ~/SafeStride
bash scripts/install_perception.sh   # 아직 하지 않았다면 최초 1회
bash scripts/run.sh
```

터미널 2에서 다음 두 토픽을 나란히 확인한다.

```bash
ros2 topic echo /perception/surface_condition
ros2 topic echo /cmd_vel_safe
```

기본 0.08 m/s 기준 기대값은 smooth 0.096, rough 0.056, block 0.052,
gravel 0.044, wet paved/unpaved mixed 0.040, mud/wet unpaved 0.032 m/s다.
snow/ice, 낮은 신뢰도, 카메라 끊김은 0 m/s다. 실제 출력은 가감속 제한 때문에
서서히 목표값에 접근하며 절대 0.15 m/s 상한을 넘지 않는다.

## 7. 최종 확인

- `/dev/safestride-drive`와 `/dev/safestride-terrain` 역할이 바뀌지 않는다.
- 두 bridge가 session을 시작하고 CRC/frame error가 계속 증가하지 않는다.
- 홀센서는 모터 트리거가 아니라 속도 피드백으로 동작한다.
- 기본 정속, 노면 배율, GPS 위치·속도가 각 ROS 토픽에 나타난다.
- enable 전에는 모터가 돌지 않고, disable·명령 timeout·USB 분리 때 정지한다.
- 실제 보행 시험 전 압력 임계값을 실제 손잡이에서 보정하고
  `PRESSURE_THRESHOLDS_CALIBRATED=true`로 바꾼다.

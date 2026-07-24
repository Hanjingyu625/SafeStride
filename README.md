# SafeStride

SafeStride는 라즈베리파이와 Arduino 호환 마이크로컨트롤러로 제어하는
2륜 스마트 보행기용 ROS 2 시작 프로젝트입니다.

라즈베리파이는 센서 정보를 처리하여 안전한 차체 속도 명령을 만들고,
마이크로컨트롤러는 바퀴 속도 제어, 출력 제한, 명령 watchdog 및 최종
모터 활성화 여부를 담당합니다.

> [!CAUTION]
> 이 저장소는 개발용 기본 구조이며 인증된 안전 제어기가 아닙니다.
> 물리적인 평상시 닫힘(NC) 비상정지는 ROS, USB, Linux 또는
> 마이크로컨트롤러 펌웨어에 의존하지 않고 모터 드라이버를 비활성화해야
> 합니다. 사람이 보행기에 몸을 의지한 상태에서는 시험하지 마십시오.
> 먼저 모터 전원을 분리한 상태에서 확인하고, 이후에는 바퀴를 들어 올린
> 상태에서 시험하십시오.

## 현재 가정한 하드웨어 구성

- 좌우에 구동 바퀴가 하나씩 있는 차동 구동 방식
- 각 구동 바퀴에 연결된 quadrature encoder
- 라즈베리파이와 마이크로컨트롤러 사이의 USB serial 연결
- 선택 사항인 전방 좌·우 거리 센서
- PWM, 방향 및 enable 입력이 있는 모터 드라이버
- active-low dead-man 스위치와 평상시 닫힘 비상정지 입력

현재 펌웨어는 고전적인 AVR Uno/Nano 계열 보드를 기준으로 작성되어
있습니다. AVR 이외의 보드로 이식하려면 먼저 영구 boot counter와
동등한 하드웨어 watchdog을 구현해야 합니다. 이를 구현하기 전에는
의도적으로 빌드가 실패하도록 되어 있습니다.

이 프로젝트에 들어 있는 형상 치수, 핀 번호, 신호 극성, encoder,
PID 및 전기적 설정값은 모두 예시입니다. 모터 전원을 연결하기 전에
반드시 다음 두 파일을 검토하십시오.

- `firmware/safestride_mcu/config.h`
- `src/safestride_bringup/config/safestride.yaml`

launch 인자인 `wheel_radius`와 `wheel_separation`은 URDF와 serial
bridge의 기구학 계산에 함께 사용되는 단일 설정값입니다.

```bash
ros2 launch safestride_bringup safestride.launch.py \
  wheel_radius:=0.15 wheel_separation:=0.55
```

## 시스템 구조

```text
/cmd_vel_intent
       |
       v
safety_supervisor ---- 거리 센서 및 상태 데이터의 최신 여부 확인
       |
       v /cmd_vel_safe
serial_bridge <====== USB serial ======> Arduino 펌웨어
       |                                  |
       |                                  +-- encoder 측정
       |                                  +-- 목표 속도 변화 제한 및 바퀴 PID
       |                                  +-- 명령 watchdog
       |                                  +-- 비상정지/dead-man/fault 상태 관리
       |
       +-- /joint_states
       +-- /odom 및 /tf
       +-- /range/front_left, /range/front_right
       +-- /battery_state
       +-- /walker/status
       +-- /diagnostics
```

serial 장치는 `serial_bridge`만 열어야 합니다. `serial_bridge`가
사용하는 속도 명령은 `safety_supervisor`만 발행하도록 구성하십시오.

## 저장소 구성

```text
src/
  safestride_interfaces/   WalkerStatus ROS 메시지
  safestride_bridge/       Serial protocol 및 ROS 하드웨어 bridge
  safestride_control/      최종 속도 명령 안전 supervisor
  safestride_bringup/      Launch 및 parameter 파일
  safestride_description/  최소 차동 구동 URDF/Xacro
firmware/
  safestride_mcu/          Arduino 호환 펌웨어
PROTOCOL.md                유선 통신 protocol 명세
```

## 1. 라즈베리파이 준비

선택한 ROS 2 배포판과 호환되는 64-bit Ubuntu를 사용하십시오.
다음 명령은 ROS 2가 이미 설치되어 있다고 가정합니다.

```bash
sudo apt update
sudo apt install python3-colcon-common-extensions python3-rosdep python3-serial

cd ~/SafeStride
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

공식 ROS 설치 안내에서 지원하는 Ubuntu와 ROS 2 배포판 조합을
사용하십시오. 서로 다른 ROS 배포판의 패키지를 섞지 마십시오.

## 2. 마이크로컨트롤러 설정 및 업로드

1. `firmware/safestride_mcu/config.h`를 엽니다.
2. 모터와 encoder 핀을 실제 배선에 맞게 설정합니다.
3. 감속비와 선택한 quadrature counting 방식을 포함하여 바퀴 출력축
   1회전당 encoder count를 입력합니다.
4. 모터 회전 방향 부호와 보수적인 PWM 제한값을 설정합니다.
5. 바퀴를 들어 올린 상태의 log를 이용해 정지 상태 arming 조건과
   encoder stall/reverse/overspeed 임계값을 조정합니다.
6. 보행기를 들어 올린 상태에서 바퀴별 PID gain을 조정합니다.
7. 비상정지와 dead-man 입력 극성이 올바른지 확인합니다.
8. `firmware/safestride_mcu/safestride_mcu.ino`를 컴파일하여 업로드합니다.

펌웨어는 Arduino core만 사용하며 별도 외부 library가 필요하지 않습니다.
Uno/Nano 보드에서도 실행할 수 있지만, 선택한 보드의 serial buffer,
RAM, interrupt 핀 및 timer/PWM 자원을 반드시 확인해야 합니다.

## 3. 고정된 serial 장치 경로 설정

마이크로컨트롤러를 연결한 후 다음 명령을 실행합니다.

```bash
ls -l /dev/serial/by-id/
```

표시된 장치 경로를
`src/safestride_bringup/config/safestride.yaml`에 입력하십시오.
재부팅 후 번호가 바뀔 수 있는 `/dev/ttyACM0` 경로는 피하는 것이 좋습니다.

기본 설정에는 `require_range_sensors: true`가 지정되어 있습니다.
실제 거리 센서 읽기 기능을 구현하기 전까지 시작 펌웨어는 `0xffff`를
전송하므로, 안전 supervisor가 의도적으로 주행을 차단합니다.
바퀴를 들어 올린 모터 단독 시험에서만 이 값을 임시로 `false`로
설정할 수 있습니다. 장애물 감지 기능을 시험하기 전에는 반드시
`true`로 되돌리십시오.

serial 장치에 접근할 수 없다면 라즈베리파이 사용자를 해당 배포판의
serial port 그룹(일반적으로 `dialout`)에 추가한 뒤 로그아웃하고
다시 로그인하십시오.

## 4. 벤치 시험 순서

첫 실행에서는 모터 전원을 분리한 상태를 유지하십시오.

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash
ros2 launch safestride_bringup safestride.launch.py
```

통신과 상태 topic을 확인합니다.

```bash
ros2 topic echo /walker/status
ros2 topic echo /diagnostics
ros2 topic hz /joint_states
```

모터 활성화를 요청하기 전에 중립 속도 명령을 계속 발행합니다.

```bash
ros2 topic pub --rate 20 /cmd_vel_intent geometry_msgs/msg/TwistStamped \
  "{header: {frame_id: base_link}, twist: {linear: {x: 0.0}, angular: {z: 0.0}}}"
```

두 번째 terminal에서 arming을 요청합니다.

```bash
ros2 service call /walker/set_enabled std_srvs/srv/SetBool "{data: true}"
```

bridge는 0이 아닌 속도 명령을 허용하기 전에 활성화 상태의 영점 명령을
여러 번 전송합니다. 펌웨어도 유효한 중립 명령을 받아야 `ARMED`
상태로 전환됩니다.

arming이 발생할 때마다 safety supervisor는 설정된 중립 deadband 안에
있는 새로운 원본 `/cmd_vel_intent`를 받을 때까지 기다립니다.
bridge도 MCU가 `ARMED` 상태임을 확인한 후 새로운 중립
`/cmd_vel_safe` 명령을 요구합니다. 따라서 joystick이 중립이 아닌
상태로 고정되어 있어도 arming 직후 갑자기 움직이지 않습니다.

장애물이 정지 거리 안으로 들어오면 즉시 영점 명령을 만들고 중립
latch를 초기화합니다. 다시 움직이려면 운전자가 입력을 중립으로
놓아야 합니다. 전진 명령을 계속 유지한 상태에서는 장애물이 사라져도
자동으로 다시 출발하지 않습니다.

모터를 비활성화하려면 다음 명령을 실행합니다.

```bash
ros2 service call /walker/set_enabled std_srvs/srv/SetBool "{data: false}"
```

## 5. 필수 고장 주입 시험

사람이나 하중을 싣기 전에 바퀴를 들어 올린 상태에서 다음 항목을
모두 확인하십시오.

- `safety_supervisor`를 종료하면 모터 목표 속도가 0이 되고 비활성화되어야 합니다.
- `serial_bridge`를 종료하면 펌웨어 watchdog이 모터를 비활성화해야 합니다.
- USB를 분리하면 펌웨어 watchdog이 모터를 비활성화해야 합니다.
- 라즈베리파이와 마이크로컨트롤러를 각각 재시작해도 모터가 자동으로
  다시 동작하면 안 됩니다.
- 주행 명령 중 비상정지를 누르면 하드웨어 enable 경로와 펌웨어 상태
  모두 정지를 표시해야 합니다.
- 비상정지를 해제해도 명시적으로 다시 arming하기 전에는 움직이면 안 됩니다.
- 손상되거나 잘린 frame, 중복 frame 및 지연된 frame은 명령 watchdog을
  갱신하면 안 됩니다.
- encoder 하나를 분리하거나 방향을 반대로 설정하고, 입력 overspeed와
  바퀴 stall을 각각 발생시켰을 때 해당 encoder fault가 latch되고
  모터 enable이 비활성 상태가 되어야 합니다. 실제 사용 전에 감시
  임계값을 조정하십시오.

보행기의 안전한 정지는 기계 시스템 전체의 특성입니다. 급격한 능동
제동은 사용자의 균형을 무너뜨릴 수 있고, 관성 주행은 경사로에서
위험할 수 있습니다. 실제 기구에서 필요한 정지 profile, holding brake,
정지 거리 및 각 fault에 대한 반응을 결정해야 합니다.

## 테스트

유선 protocol test는 ROS 없이 실행할 수 있습니다.

```bash
PYTHONPATH=src/safestride_bridge \
  python3 -m unittest discover -s src/safestride_bridge/test -v
```

C++ 펌웨어와 Python bridge가 동일한 byte를 만드는지는
cross-language golden vector test로 확인합니다.

```bash
g++ -std=c++11 \
  -Itest/arduino_stub -Ifirmware/safestride_mcu \
  test/firmware_protocol_test.cpp firmware/safestride_mcu/protocol.cpp \
  -o /tmp/safestride_protocol_test
/tmp/safestride_protocol_test
```

host-side motor test는 정상 feedback과 latch되는 no-edge,
reverse-direction 및 overspeed fault를 검사합니다.

```bash
g++ -std=c++11 \
  -Itest/arduino_stub -Ifirmware/safestride_mcu \
  test/firmware_motor_control_test.cpp \
  firmware/safestride_mcu/motor_control.cpp \
  -o /tmp/safestride_motor_test
/tmp/safestride_motor_test
```

state-machine test는 watchdog 만료 시 현재 session이 무효화되고,
새로운 HELLO handshake와 비활성화 reset이 끝날 때까지 대기 중인
명령을 거부하는지 확인합니다.

```bash
g++ -std=c++11 \
  -Itest/arduino_stub -Ifirmware/safestride_mcu \
  test/firmware_state_machine_test.cpp \
  firmware/safestride_mcu/protocol.cpp \
  firmware/safestride_mcu/motor_control.cpp \
  -o /tmp/safestride_state_test
/tmp/safestride_state_test
```

ROS 의존성을 설치한 후에는 다음 명령으로 전체 test를 실행합니다.

```bash
colcon test
colcon test-result --verbose
```

## 실제 하드웨어에 맞춰 추가할 작업

지상 주행을 시작하기 전에 placeholder 센서 읽기 함수를 실제 센서
코드로 교체하고, 바퀴 반지름과 좌우 바퀴 간격을 보정해야 합니다.
각 바퀴의 속도 제어기와 encoder plausibility monitor를 개별적으로
조정하고, 모터 전류 및 driver fault 입력을 추가한 뒤 모든 fault bit에
대한 반응을 정의하십시오. 이후에는 보정된 IMU와 encoder odometry를
융합하는 것을 권장합니다.

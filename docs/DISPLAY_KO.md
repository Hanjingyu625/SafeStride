# ezHMI 디스플레이 설치·연결·배선 가이드

이 문서는 Windows PC에서 ezHMI 화면을 준비하고, LCD를 단독으로 확인한 뒤
SafeStride Terrain Uno에 연결하는 절차를 설명한다.

대상 하드웨어는 다음과 같다.

- ezHMI `EZ48270M043-LE`, 480 × 272, 가로 화면
- M-LE 16P 인터페이스 보드의 TTL UART
- Arduino Uno ATmega328P 기반 Terrain MCU
- Raspberry Pi 4, ROS 2 Jazzy 기반 SafeStride

`display/ezhmi/visualtft/Project.tftprj`는 바로 열어 컴파일할 수 있는 VisualTFT
3.0 네이티브 소스 프로젝트다. `prototypes/ezhmi_display/gui/safestride-cockpit.html`은
디자인 확인용 브라우저 시뮬레이터이며 LCD 다운로드 파일은 아니다.

저장소에는 화면, Modbus 변수, Lua와 필요한 프로젝트 글꼴 리소스가 포함되어 있다.
VisualTFT가 생성하는 `output`, `script.map` 및 SD 패키지는 재생성 가능하므로 Git에
포함하지 않는다. 이 개발 환경에서 XML·컨트롤·레지스터·Lua 일치 검증은 수행하지만,
실제 VisualTFT 컴파일·화면 출력·LCD 다운로드는 Windows 도구와 실물에서 확인해야 한다.

## 1. 전체 데이터 경로

```text
ROS 2 topics
  /walker/status, /handle/pressure, /crosswalk/status, /terrain/status
       |
       v
Raspberry Pi TerrainBridgeNode
       |  USB serial, protocol v6, 115200 baud
       v
Terrain Uno hardware Serial (D0/D1 is reserved for USB)
       |  AltSoftSerial, Modbus RTU, 19200 8N1
       |  Uno D9 TX / D8 RX
       v
M-LE 16P board TTL DIN/DOUT
       |
       v
EZ48270M043-LE LCD
```

Terrain Uno가 LCD에 표시 상태만 전송한다. LCD와 Lua 스크립트는 모터 명령을
보내지 않으며 DEV 버튼도 비활성으로 구성한다.

## 2. 연결 전에 준비할 물품

- 모델과 터치 형식이 주문 내용과 일치하는 `EZ48270M043-LE`
- M-LE 16P 인터페이스 보드와 LCD 연결 케이블
- LCD 소비전류에 여유가 있는 안정화 5 V 전원
- GND, TX, RX 연결용 배선
- LCD 프로젝트 다운로드용 USB-UART 인터페이스 또는 FAT32 microSD 카드
- Windows 10/11 PC
- Arduino Uno용 USB 케이블
- 가능하면 멀티미터와 USB-UART 로직 레벨 확인 장비

제품 라벨, 16P 보드의 `VCC/GND/DIN/DOUT` 표기와 입력 전압을 먼저 사진 또는
제품 문서와 대조한다. 이 문서의 Terrain 연결은 TTL UART용이다. `RS232`라고
표기된 단자에는 Uno 핀을 직접 연결하지 않는다.

LCD 전원은 별도의 안정화 5 V 출력을 사용한다. LCD에 USB 전원이 들어오는 상태에서
같은 VCC에 외부 5 V를 병렬로 연결하지 않는다. PC 다운로드 인터페이스가 LCD에
전원을 공급하는지 확실하지 않으면 VCC를 연결하기 전에 멀티미터로 확인한다.

## 3. Windows에 설치할 소프트웨어

### 3.1 Git

Git for Windows를 설치하고 PowerShell 또는 Git Bash에서 저장소를 받는다.

```powershell
git clone --branch pdj1 --single-branch https://github.com/Hanjingyu625/SafeStride.git
cd SafeStride
git status --short --branch
```

이미 clone한 저장소가 있으면 작업 중인 파일을 먼저 보존한 다음 갱신한다.

```powershell
git fetch origin
git switch pdj1
git pull --ff-only origin pdj1
```

### 3.2 VisualTFT

LCD 제조사 또는 판매처가 제공하는 VisualTFT를 Windows에 설치한다. 임의의
제3자 다운로드 사이트보다 제조사·판매처 배포본을 우선한다. 설치 후 새 프로젝트에서
M series와 480 × 272 모델을 선택한다. 모델 목록에 여러 4.3인치 항목이 있으면
LCD 뒷면 라벨과 판매처 자료를 기준으로 `480272M043` 계열을 선택한다.

VisualTFT 제조사 사용 문서:

- 설치와 작업 화면: <https://doc.gz-dc.com/QuickStart/03_PCsoft.html>
- M시리즈 프로젝트 다운로드: <https://doc.gz-dc.com/QuickStart/05_download.html>
- Modbus 기본 설정: <https://doc.gz-dc.com/Modbus/Ctrls/01_Mb_basic.html>
- Modbus 변수 생성: <https://doc.gz-dc.com/Modbus/Ctrls/02_Mb_CreateVariables.html>

USB-UART 또는 LCD 다운로드 보드가 Windows 장치 관리자에서 COM 포트로 보이지
않으면 보드에 실장된 USB-UART 칩을 확인하고 해당 제조사 드라이버를 설치한다.
칩 종류를 확인하지 않은 상태에서 임의의 드라이버를 설치하지 않는다.

### 3.3 Arduino 개발 환경

Arduino IDE 2 또는 `arduino-cli`를 설치한다. Terrain 펌웨어는 Arduino AVR Boards와
AltSoftSerial 1.4가 필요하다.

Arduino IDE에서는 Library Manager에서 `AltSoftSerial`을 검색해 Paul Stoffregen의
라이브러리를 설치한다. `arduino-cli`를 사용하면 다음 명령으로 준비한다.

```powershell
arduino-cli core update-index
arduino-cli core install arduino:avr
arduino-cli lib install "AltSoftSerial@1.4.0"
arduino-cli core list
arduino-cli lib list
```

## 4. VisualTFT 프로젝트 만들기

### 4.1 화면과 컨트롤

VisualTFT에서 다음 네이티브 프로젝트를 연다.

- `display/ezhmi/visualtft/Project.tftprj`

아래 파일은 프로젝트가 따라야 하는 원본 명세와 디자인 참고 자료다.

- `display/ezhmi/controls.csv`: 화면 번호, 컨트롤 ID, 위치, 크기와 초기 문구
- `display/ezhmi/registers.csv`: Modbus 레지스터 이름, 주소와 의미
- `display/ezhmi/safestride.lua`: 부팅, 통신 끊김과 상태 표시 로직
- `prototypes/ezhmi_display/gui/safestride-cockpit.html`: 시각 디자인 미리보기
- `prototypes/ezhmi_display/gui/UI_LAYOUT_KO.md`: 문구와 상태 해석 원칙

포함된 프로젝트에는 화면 두 개가 구성되어 있다.

1. 화면 `0`: SafeStride 로고 부팅 화면
2. 화면 `1`: 속도, 주행 여부, 도로 상황과 경사 상태 화면

`controls.csv`의 기능 ID 1~16은 Lua에서 직접 사용하므로 임의로 바꾸지 않는다.
ID 17~21은 카드 배경과 하단 상태 바다. DEV 버튼은 화면 `1`, ID `11`이며
동작·송신 데이터가 없고 Lua도 비활성화한다.

VisualTFT 기본 글꼴의 한글이 가상 화면에서 `?`로 대체되는 것을 확인했으므로 실제
LCD 문구는 ASCII 영문으로 구성한다. UTF-8 한글 글꼴을 별도로 검증하기 전에는
Screen 파일이나 Lua에 한글 표시 문자열을 다시 넣지 않는다.

### 4.2 Modbus RTU 설정

VisualTFT의 `도구 → 프로토콜 및 변수 설정`에서 다음과 같이 구성한다.

| 항목 | 설정 |
|---|---|
| 프로토콜 | Modbus RTU |
| LCD 역할 | Slave |
| Slave ID | `1` |
| UART | TTL, `19200 baud`, `8 data bits`, `no parity`, `1 stop bit` |
| 변수 형식 | Holding Register, unsigned 16-bit word |
| 시작 주소 | `0x0000` |
| 마지막 주소 | `0x000F` |
| 호스트 쓰기 | Function Code `0x10`, Write Multiple Registers 허용 |

16개 변수를 정확히 다음 주소와 이름으로 만든다.

| 주소 | VisualTFT 변수 | 값 |
|---:|---|---|
| `0000` | `ss_version` | snapshot 버전, 현재 `2` |
| `0001` | `ss_heartbeat` | 200 ms마다 변하는 생존 카운터 |
| `0002` | `ss_valid` | speed=1, hands=2, crosswalk=4, pitch=8, ToF=16, walker=32 |
| `0003` | `ss_speed` | km/h × 100, `65535`는 무효 |
| `0004` | `ss_hands` | 왼쪽=1, 오른쪽=2 |
| `0005` | `ss_crosswalk` | CrosswalkStatus 상태 0..6 |
| `0006` | `ss_seconds` | 신호 잔여 초, `65535`는 무효 |
| `0007` | `ss_distance` | 횡단보도 경계 거리 m × 10, `65535`는 무효 |
| `0008` | `ss_pitch` | int16 2의 보수, degree × 10 |
| `0009` | `ss_tof` | ToF 상태 0..5, 5는 무효 |
| `000A` | `ss_hazard` | 확정 위험 latch |
| `000B` | `ss_walker` | WalkerStatus 상태 0..5 |
| `000C` | `ss_braking` | 제동 중이면 1 |
| `000D` | `ss_faults` | Drive fault bit mask |
| `000E` | `ss_host_link` | Pi snapshot이 fresh하면 1 |
| `000F` | `ss_flags` | armed=1, deadman=2, estop=4, watchdog=8, entry=16, urgent=32, signal=64 |

주소는 VisualTFT에서 16진수로 입력한다. `ss_pitch`는 레지스터 자체는 unsigned
16-bit로 만들고 Lua에서 2의 보수 signed 값으로 변환한다. `40001` 방식의 표시
주소를 요구하는 도구가 있더라도 실제 Modbus PDU 시작 주소는 `0x0000`이어야 한다.

모든 변수는 읽기/쓰기 허용, 배율 1, Flash 저장 비활성으로 설정한다.
기본값은 `registers.csv`의 `default` 열을 따른다. 특히 `ss_version=2`,
`ss_valid=0`, `ss_host_link=0`, `ss_speed/ss_seconds/ss_distance=65535`로
초기화해야 출고·재부팅 시 예전 주행 값을 정상 상태처럼 표시하지 않는다.
텍스트 컨트롤은 문자열 모드로 만들고 Lua가 갱신하도록 자동 숫자 바인딩을 끈다.

### 4.3 Lua 추가

네이티브 프로젝트의 `main.lua`에는 `display/ezhmi/safestride.lua` 내용이 반영되어
있다. 후자를 편집 가능한 UTF-8 원본으로 유지하며 두 파일의 내용이 일치해야 한다.
프로젝트에서 사용하는 API 이름이 다음과 일치하는지 VisualTFT 컴파일 결과로 확인한다.

- `set_text`
- `set_fore_color`
- `set_enable`
- `change_screen`
- `start_timer`
- `get_variant`

스크립트는 부팅 후 2.2초에 상태 화면으로 이동한다. heartbeat가 약 1초 동안
변하지 않거나 `ss_host_link`가 0이면 녹색 주행 상태를 지우고
`Link lost / waiting`을 표시한다. 이 fail-safe 표시 동작을 삭제하지 않는다.

### 4.4 가상 화면 검사

VisualTFT에서 프로젝트를 컴파일하고 가상 화면을 실행한다. 최소한 다음 상태를
각각 확인한다.

- 최초 부팅 로고와 2.2초 뒤 화면 전환
- 레지스터 초기값일 때 `Waiting for link`
- 정상 속도와 양손 감지
- 제동 및 위험 감지
- 횡단보도 N/A, 대기와 진입 가능
- 양수·음수 pitch와 급경사
- heartbeat 정지 시 약 1초 안에 통신 끊김 표시
- DEV 버튼이 눌리지 않음

HTML 시뮬레이터에서 보이는 `서울시 광진구`는 예시 데이터다. 현재 HMI packet에는
위치 문자열이 없으므로 실제 LCD에는 `위치: N/A`가 정상이다.

## 5. Terrain과 연결하기 전에 LCD 단독 다운로드

먼저 Terrain Uno와 LCD UART를 연결하지 않은 상태에서 프로젝트를 LCD에 넣는다.

### UART 다운로드

1. 16P 보드와 LCD 케이블 방향을 확인한다.
2. 다운로드 보드의 TX/RX/GND를 해당 보드 설명에 맞게 연결한다.
3. 전원 공급 경로가 하나뿐인지 확인하고 LCD에 전원을 넣는다.
4. Windows 장치 관리자에서 COM 포트를 확인한다.
5. VisualTFT에서 프로젝트를 컴파일하고 M시리즈 UART 다운로드를 실행한다.
6. 완료 메시지 후 LCD를 재부팅한다.

### SD 카드 다운로드

M시리즈와 실제 16P 보드가 SD 업데이트를 지원하면 다음 절차를 사용할 수 있다.

1. 8 GB 이하 SD 카드를 FAT32로 포맷한다.
2. VisualTFT의 M시리즈 양산 도구로 SD upgrade package를 생성한다.
3. 생성된 `SD_PACKET` 디렉터리의 파일을 SD 카드 루트에 복사한다.
4. LCD 전원을 끄고 SD 카드를 삽입한다.
5. LCD 전원을 켜고 `update finished`가 표시될 때까지 기다린다.
6. 자동 재부팅 또는 전원 차단 후 SD 카드를 제거한다.

LCD 단독 부팅에서 SafeStride 인트로, 상태 화면과 영문 문구가 정상인지 확인한다.
Terrain이 없으므로 상태 화면에 `Link lost / waiting`이 나오는 것이 정상이다.

## 6. Terrain 펌웨어 준비

현재 HMI 통합 펌웨어는 `firmware/terrain_mcu`에 있다. 컴파일 전에 다음을 확인한다.

```powershell
arduino-cli lib list
arduino-cli compile --fqbn arduino:avr:uno firmware/terrain_mcu
```

VisualTFT 프로젝트의 slave ID, 시작 주소와 baudrate가 확정되기 전에는
`firmware/terrain_mcu/ezhmi_transport.h`의 `EZHMI_ENABLED`를 `0`으로 빌드한다.
이 문서의 설정과 일치하는 프로젝트를 LCD에 다운로드하고 가상 화면 검사를 통과한
뒤에만 `EZHMI_ENABLED=1`을 사용한다.

운영 코드 기본값은 `1`이다. LCD 설정 전 UART 출력을 끈 빌드는 다음과 같다.

```powershell
arduino-cli compile --fqbn arduino:avr:uno --build-property compiler.cpp.extra_flags=-DEZHMI_ENABLED=0 firmware/terrain_mcu
```

Terrain Uno 업로드 예시는 다음과 같다. Windows의 실제 COM 포트로 바꾼다.

```powershell
arduino-cli upload --fqbn arduino:avr:uno --port COM5 firmware/terrain_mcu
```

업로드 중에는 Arduino Serial Monitor와 ROS bridge가 같은 Uno serial 포트를 열고
있으면 안 된다.

### 6.1 Pi 코드 적용

Git push는 Pi/Uno/LCD를 자동 업데이트하지 않는다. Pi의 작업 중인 변경을 보존하고
최신 `pdj1`을 반영한 다음 기존 ROS 실행을 종료하고 아래처럼 빌드한다.
Terrain 펌웨어와 Pi bridge는 함께 업데이트한다. 이미 protocol v6/schema 0x0601인
Drive 펌웨어는 이번 디스플레이 변경 때문에 재업로드할 필요가 없다.

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to safestride_bridge safestride_bringup
source install/setup.bash
```

두 운영 YAML의 `terrain_bridge.ros__parameters.hmi` 기본값은
`enabled=true`, `pitch_sign=-1.0`, `pitch_offset_rad=0.0`이다.
경사 보정 변경 시 safety supervisor의 `uphill_pitch_sign`과 `pitch_offset_rad`도
일치시킨다. `/terrain/status`의 raw pitch를 LCD relay가 한 번 보정한다.

Pi는 기존 TerrainBridgeNode의 USB 연결만 사용하여 200 ms마다 32바이트
HMI snapshot(`0x30`)을 보낸다. 별도 serial 프로세스를 실행하지 않는다.
HMI v2 capability는 bit 11이며, bit 10의 과거 초안과 구별한다.
Terrain은 600 ms 동안 snapshot이 없으면 데이터를 무효화하며 LCD는 자체
heartbeat 감시로 Terrain 전원이 꺼져도 1초 내 연결 끊김을 표시한다.
LCD 상태(`0x31`, `<BBHII`, 12 bytes)는 버전 2, ACK 유효 여부, exception code,
누적 ACK 수, 누적 오류 수다. 센서 telemetry의 기존 payload는 유지한다.

## 7. Terrain Uno와 LCD 배선

모든 전원을 끈 상태에서 연결한다.

| Terrain Uno | M-LE 16P TTL I/F | 설명 |
|---|---|---|
| D9, TX | DIN 또는 RX | Uno가 LCD로 전송 |
| D8, RX | DOUT 또는 TX | LCD 응답을 Uno가 수신 |
| GND | GND | Pi, Uno, LCD 공통 신호 기준 |
| 연결하지 않음 | VCC | Uno 5 V 핀에서 LCD 전원을 공급하지 않음 |
| 별도 5 V 전원 `+` | VCC | LCD용 안정화 전원 |
| 별도 5 V 전원 `-` | GND | 공통 GND에 연결 |

```text
Terrain Uno D9 (TX)  ----------------> LCD DIN/RX
Terrain Uno D8 (RX)  <---------------- LCD DOUT/TX
Terrain Uno GND      ----------------- LCD GND ---- 5 V supply negative
                                                     5 V supply positive ---- LCD VCC
```

AltSoftSerial은 Uno ATmega328P에서 D8=RX, D9=TX로 고정된다. Timer1을 사용하며
PWM D10을 사용할 수 없다. Terrain Uno의 TOF-10120과 MPU6050은 A4/A5 I2C를
사용하고 Pi 연결은 USB hardware Serial을 사용하므로 현재 핀 구성과 충돌하지 않는다.

모터 전원선과 LCD UART 배선을 떨어뜨리고, 긴 UART 배선은 피한다. 커넥터가 당겨지지
않도록 서비스 루프와 strain relief를 둔다.

## 8. 최초 전원 인가와 확인 순서

1. 모터드라이버 12 V 전원을 분리한다.
2. Terrain Uno와 LCD 사이 VCC가 연결되지 않았고 공통 GND만 공유하는지 확인한다.
3. LCD 별도 5 V의 극성과 무부하 전압을 측정한다.
4. LCD만 켜서 `Link lost / waiting` 화면을 확인한다.
5. 전원을 모두 끈다.
6. D9→DIN/RX, D8←DOUT/TX를 연결한다.
7. Pi와 Terrain Uno를 USB로 연결한다.
8. LCD 5 V와 Pi/Terrain 전원을 켠다.
9. ROS를 모터 비활성 상태로 실행한다.
10. LCD가 실제 상태로 바뀌고 통신 진단 ACK가 증가하는지 확인한다.

Pi에서 확인한다.

```bash
cd ~/SafeStride
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42

ros2 topic echo /terrain/status --once
ros2 topic echo /walker/status --once
ros2 topic echo /diagnostics
```

`/diagnostics`의 `SafeStride ezHMI display` 항목은 정상 ACK가 있으면
`LCD Modbus ACK active`를 표시한다. 이 진단은 UART와 Modbus 응답만 확인하며
화면 배치나 문구가 올바른지는 보장하지 않는다.

## 9. 증상별 확인

| 증상 | 확인 사항 |
|---|---|
| LCD가 켜지지 않음 | 별도 5 V 극성·전압, 전원 전류 여유, 16P 케이블 방향 |
| Windows COM 포트 없음 | USB-UART 칩과 전용 드라이버, 케이블의 데이터 지원 여부 |
| 프로젝트 다운로드 실패 | 정확한 M시리즈 모델, COM 포트 점유, 다운로드 보드 TX/RX/GND |
| 문구가 `?`로 표시됨 | 저장소의 ASCII 영문 프로젝트를 다시 열고 컴파일했는지 확인 |
| 계속 `Waiting for link` | ROS 실행, Terrain USB serial, HMI capability와 `hmi.enabled` |
| LCD ACK 없음 | TTL/RS232 단자 구분, TX/RX 교차, 공통 GND, slave ID와 19200 8N1 |
| Modbus exception | holding register 0x0000..0x000F와 FC16 허용 여부 |
| 값은 바뀌지만 문구가 이상함 | 변수 이름·주소·uint16 형식, Lua와 control ID |
| 경사 부호가 반대 | `hmi.pitch_sign`과 safety supervisor의 pitch 부호 설정 일치 여부 |
| 잠시 정상 후 연결 끊김 | 5 V 강하, UART 배선 노이즈, Pi/Terrain serial 재연결 기록 |

## 10. 최종 체크리스트

- [ ] LCD 라벨과 16P 인터페이스 보드 모델을 확인했다.
- [ ] 저장소의 VisualTFT M series 480 × 272 프로젝트를 열고 컴파일했다.
- [ ] Modbus slave 1, 19200 8N1, holding register 0000..000F를 설정했다.
- [ ] `controls.csv`, `registers.csv`, `safestride.lua`를 프로젝트에 반영했다.
- [ ] 영문 문구와 모든 상태를 VisualTFT 가상 화면에서 확인했다.
- [ ] Terrain과 분리된 LCD에 프로젝트를 다운로드했다.
- [ ] LCD 단독 부팅에서 `Link lost / waiting` 화면을 확인했다.
- [ ] AltSoftSerial 1.4를 설치하고 Terrain 펌웨어를 컴파일했다.
- [ ] LCD 전원은 별도 5 V이고 USB 5 V와 병렬 연결되지 않는다.
- [ ] D9→DIN/RX, D8←DOUT/TX, 공통 GND를 전원 OFF 상태에서 연결했다.
- [ ] 모터 12 V를 분리한 상태에서 ROS topic과 LCD ACK를 확인했다.
- [ ] 마지막에만 제한된 실물 주행 시험으로 넘어간다.

## 11. 코드 검증

```bash
bash scripts/test_firmware.sh
PYTHONPATH=src/safestride_bridge python3 -m unittest discover -s src/safestride_bridge/test
python3 -m unittest discover -s test -p test_display_lua.py
arduino-cli compile --fqbn arduino:avr:uno firmware/terrain_mcu
```

Lua 테스트는 system liblua5.3으로 제조사 API를 mock하여 로고 전환, 데이터
무효화, 제동/진입 허가 문구를 검사한다. 라이브러리가 없으면 skip된다.
ROS가 없는 환경에서는 실제 ROS 통합 테스트 2건도 skip되므로 ROS Jazzy의
빌드된 workspace에서 검사한다. 이 테스트들은 실제 VisualTFT/LCD 검증을 대신하지 않는다.

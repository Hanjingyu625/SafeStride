# ezHMI 디스플레이 초안

대상 LCD: ezHMI EZ48270M043-LE + M-LE 16P 인터페이스 보드

## 최신 UI (v2)

`gui/safestride-cockpit.html`을 브라우저로 열면 사용자 스케치를 반영한 최신 UI를 볼 수 있다.
왼쪽은 속도·주행 여부, 오른쪽은 도로 상황·경사 정보이며 DEV 모드는 비활성 버튼만 있다.
전원 OFF → ON 시 2.2초 인트로를 재생한다. 위치와 수치는 예시 데이터다.
편집용 원본은 `gui/safestride-cockpit.fragment.html`, 설계 설명은 `gui/UI_LAYOUT_KO.md`다.
기존 `gui/ezhmi-display-simulator.html`은 v1 비교용으로 보관한다.

이 폴더는 2026-09-13 작성한 독립 초안이며 운영 firmware/src에 적용되지 않았다.
복사된 Terrain/Pi 코드는 당시 기준이다. 적용 시 최신 운영 코드의 변경점과 병합해야 하며
파일 전체를 덮어쓰지 않는다. 특히 pi/terrain_bridge_node.py는 기존 ROS 패키지의
validation 모듈 등에 의존하므로 이 폴더에서 단독 실행하는 프로그램이 아니다.

LCD Modbus 주소·VisualTFT 프로젝트는 미확정이며 전송은 기본 비활성이다.
AltSoftSerial 1.4는 Arduino Library Manager 또는
https://github.com/PaulStoffregen/AltSoftSerial 에서 설치한다.
외부 라이브러리 ZIP과 Python 캐시, 빌드 산출물은 커밋에 포함하지 않는다.

## 화면 동작

- 전원 인가: v2 미리보기는 `SafeStride` 인트로를 2.2초 표시한 뒤 상태 화면으로 전환
- 현재 속도: `/walker/status`의 `measured_speed_kmh`
- 손잡이: `/handle/pressure`의 좌·우 압력 상태
- 횡단보도: `/crosswalk/status`; GPS/상태가 유효하고 접근 중일 때만 표시하며 그 밖에는 `N/A`
- 경사: `/terrain/status`의 보정된 pitch
- 전방 위험: Terrain ToF의 confirmed raised/drop 또는 `terrain_hazard`
- 추가: 통신 상태, 센서 유효성, DRIVE/BRAKING, fault bits

## 연결 초안

Terrain Uno의 기존 Pi UART(하드웨어 `Serial`)는 유지한다. LCD는 Uno ATmega328P에서
AltSoftSerial을 사용한다.

| Terrain Uno | 16P 보드 TTL I/F | 비고 |
|---|---|---|
| D9 (TX) | DIN/RX | 3.3~5 V TTL |
| D8 (RX) | DOUT/TX | 교차 연결 |
| GND | GND | Pi/Uno/LCD 공통 기준 |
| 별도 5 V 안정 전원 | VCC | LCD에 5 V 직결, USB 5 V와 병렬 연결 금지 |

AltSoftSerial은 Uno에서 D8=RX, D9=TX를 고정으로 사용하며 Timer1과 PWM D10을 점유한다.
모터 전원선과 LCD UART선을 분리하고, LCD 커넥터에는 서비스 루프를 남긴다.

## 통신 초안

Pi의 기존 TerrainBridgeNode가 세션을 연 뒤 200 ms마다 32바이트 little-endian 상태
snapshot(`16 x uint16`)을 Terrain Uno로 전송한다. Terrain Uno는 이를 실험용 packet
type `0x30`으로 받아 LCD용 Modbus RTU FC16 프레임으로 변환한다.

레지스터 의미는 `firmware/terrain_mcu/hmi_state.h`의 `Index`를 따른다. 이는 프로젝트
내부 초안 주소이며 ezHMI 실제 변수/레지스터 주소가 아니다. 제조사 VisualTFT 프로젝트에서
다음 항목을 확인한 후 `EZHMI_MODBUS_MAP_CONFIRMED=1`로 빌드한다.

1. Modbus slave ID, 시작 레지스터 주소와 각 레지스터의 표시 형식
2. 19200 8N1 또는 실제 UART 설정, FC16 응답 형식
3. 부팅 화면/스크립트와 화면 변수 다운로드 방법
4. 16P 보드의 TTL 핀 방향과 LCD의 별도 5 V 전류 여유

그 전까지는 LCD에 임의의 제조사 명령을 보내지 않도록 전송부가 비활성화되어 있다.

## 검토 방법

- 배송 전 UI: `gui/ezhmi-display-simulator.html` 또는 대화창의 시뮬레이터에서 전원, 정상,
  횡단보도, 전방 위험, 통신 끊김 상태를 확인한다.
- Python 문법: `python -m py_compile pi/*.py`
- Arduino 빌드: `arduino-cli compile --fqbn arduino:avr:uno firmware/terrain_mcu`

이 초안은 화면·데이터 경로 계획이며, LCD 실물의 통신 주소가 확인되기 전에는 플래시하거나
주행 안전 기능으로 간주하지 않는다.

# Terrain MCU 센서 단독 테스트

대상은 **Terrain Arduino Uno 한 대**이다. TOF-10120, MPU-9250 내부의
가속도/자이로와 AK8963 지자기센서, BNO055를 하나의 I2C 버스에서 확인한다.
다리 액추에이터 출력은 사용하지 않는다.

## 핀맵

| Uno 핀 | 연결 대상 | 방향/동작 | 비고 |
|---|---|---|---|
| A4/SDA | TOF, MPU-9250, BNO055의 SDA | I2C 데이터 | 모든 센서 병렬 연결 |
| A5/SCL | TOF, MPU-9250, BNO055의 SCL | I2C 클록 | 모든 센서 병렬 연결 |
| 5V/3.3V | 각 센서 모듈 전원 | 전원 | 모듈 사양에 맞춰 선택 |
| GND | 모든 센서 GND | 공통 기준 | Uno와 반드시 공통 연결 |

| 장치 | 예상 7-bit I2C 주소 | 정상 확인 |
|---|---:|---|
| TOF-10120 | `0x52` | 거리값 출력 |
| MPU-9250 | `0x68` 또는 `0x69` | WHO_AM_I 및 가속도/자이로 출력 |
| AK8963 | `0x0C` | MPU bypass 활성화 후 지자기 출력 |
| BNO055 | `0x28` 또는 `0x29` | 자세, 보정도, 시스템 상태 출력 |

Uno의 I2C 신호는 5 V 계열이다. **원시 3.3 V 센서 보드에는 직접 연결하지
말고**, 사용 중인 브레이크아웃 모듈의 전원 입력, 풀업 저항과 레벨시프터
지원 여부를 확인한다. 여러 모듈에 풀업 저항이 중복되어 통신이 불안정하면
모듈 회로도에 따라 풀업 구성을 조정한다.

## 업로드

Terrain Uno의 실제 포트를 확인해 `<TERRAIN_PORT>`를 바꾼다. SafeStride
Uno의 포트를 사용하면 다른 보드의 펌웨어를 덮어쓰므로 두 USB 케이블을
한 번씩 분리하며 포트를 식별하는 것이 안전하다.

```powershell
arduino-cli compile --fqbn arduino:avr:uno firmware/terrain_sensor_bench_test
arduino-cli upload --fqbn arduino:avr:uno -p <TERRAIN_PORT> firmware/terrain_sensor_bench_test
```

Arduino IDE를 사용할 경우
`firmware/terrain_sensor_bench_test/terrain_sensor_bench_test.ino`를 열고
보드를 `Arduino Uno`, 포트를 Terrain Uno로 선택한 뒤 업로드한다.

## 테스트 방법

1. 다리 액추에이터와 모터 전원을 분리한다.
2. 센서들을 A4/A5 I2C 버스와 공통 GND에 연결하고 전압을 다시 확인한다.
3. 시리얼 모니터를 `115200 baud`, 줄바꿈 `Newline`으로 연다.
4. `SCAN`을 입력해 위 표의 주소가 모두 보이는지 확인한다.
5. 누락된 주소가 있으면 전원을 끈 뒤 해당 모듈의 전압, SDA/SCL 뒤바뀜,
   주소 선택 핀과 공통 GND부터 확인한다.
6. `REINIT`으로 IMU들을 다시 초기화하고 `STATUS`를 입력한다.
7. TOF 앞 물체를 이동해 `tof_raw_mm`, `tof_filtered_mm`가 변하는지 확인한다.
   유효 범위는 현재 운영 설정과 같은 100~2000 mm이다.
8. 보드를 천천히 기울이고 회전해 MPU의 `accel`, `gyro`, `mag` 값과 BNO의
   `heading_deg`, `roll_deg`, `pitch_deg`가 변하는지 확인한다.
9. BNO055의 `cal_sys`, `cal_gyr`, `cal_acc`, `cal_mag`는 각각 0~3이며,
   충분히 움직여 보정한 뒤 3에 가까워지는지 확인한다. `sys_error=0`이
   정상이다.

사용 가능한 명령은 다음과 같다.

```text
STATUS
SCAN
REINIT
STREAM ON
STREAM OFF
HELP
```

- `STATUS`: 전체 센서 상태를 한 번 출력한다.
- `SCAN`: I2C 버스에서 응답하는 주소를 출력한다.
- `REINIT`: MPU-9250/AK8963/BNO055를 다시 탐색하고 초기화한다.
- `STREAM ON/OFF`: 200 ms 주기 연속 출력을 켜거나 끈다.

TOF 출력 판정은 다음과 같다.

| `tof_alert` | 의미 |
|---|---|
| `NORMAL` | 기준면과 유사 |
| `CANDIDATE` | 단차 후보 |
| `STEP` | 연속 조건을 만족한 단차 |
| `INVALID` | 통신 실패 또는 범위 밖 |

상태 LED는 사용하지 않는다. 모든 판정과 오류는 시리얼 출력에서 확인한다.

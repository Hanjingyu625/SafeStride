# Arduino 센서 테스트벤치 안내

두 Arduino는 서로 다른 보드이며, 반드시 각각 따로 업로드하고 테스트한다.

| 보드 | 담당 장치 | 테스트 문서 |
|---|---|---|
| SafeStride MCU (Drive Uno) | 압력센서, 비상정지, 좌우 홀센서, 모터드라이버 상태 | [SafeStride 보드 테스트](safestride_sensor_bench_test/README.md) |
| Terrain MCU (Terrain Uno) | TOF-10120, MPU-9250/AK8963, BNO055 | [Terrain 보드 테스트](terrain_sensor_bench_test/README.md) |

두 스케치는 Arduino Uno 기준이며 시리얼 속도는 `115200 baud`이다. 테스트벤치
업로드 시 해당 보드의 기존 펌웨어가 덮어써지므로 통합 시험 전에는 운영
펌웨어를 다시 업로드해야 한다.

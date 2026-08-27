# Architecture

```text
Drive Uno telemetry ----> serial_bridge ----+
                                            +-> safety_supervisor -> Drive Uno command
Terrain Uno telemetry --> terrain_bridge ---+
  TOF / MPU6050 / GPS          |
                              +-> diagnostics / Foxglove / crosswalk monitor
```

Drive Uno가 PWM, command watchdog, Hall plausibility와 압력 dead-man을 최종
집행한다. Raspberry Pi safety supervisor는 Drive 상태와 Terrain TOF 상태가
신선하고 유효할 때만 명령을 전달한다.

TOF 확정 hazard, serial timeout, invalid TOF, dead-man 해제 또는 Drive MCU fault가
발생하면 ROS는 0 명령을 즉시 발행하고 이후 송신을 억제한다. 다시 움직이려면
hazard가 사라진 뒤 `/walker/set_enabled`로 명시적으로 활성화해야 한다. MPU와
GPS 오류는 각 bridge 진단에 표시하지만 단독으로 모터를 차단하지 않는다.

GPS와 횡단보도 기능은 기본적으로 관찰 전용이다. 지도와 API가 없어도 진단을
발행하지만 motion command 경로에는 참여하지 않는다. Foxglove도 기본 비활성이며
활성화할 때 publish/service/parameter 기능을 제한한 읽기 전용 설정을 사용한다.

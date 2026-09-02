# Architecture

```text
Drive Uno telemetry ----> serial_bridge ----+
                                            +-> safety_supervisor -> Drive Uno command
Terrain Uno telemetry --> terrain_bridge ---+
  TOF / MPU6050                |
BE-220 GPS -> Pi gps_node -----+-> diagnostics / Foxglove / crosswalk monitor
```

Drive Uno가 PWM, command watchdog, Hall plausibility와 압력 dead-man을 최종
집행한다. Raspberry Pi safety supervisor는 Drive 상태와 Terrain TOF 상태가
신선하고 유효할 때만 명령을 전달한다.

TOF 확정 hazard, serial timeout, invalid TOF 또는 Drive MCU fault가 발생하면
즉시 정지한다. 정상적인 dead-man 해제만 Drive MCU가 목표속도를 0.6초 동안
ramp한 뒤 정지한다. 정상 링크, fresh 안전 명령, Hall/TOF와 양손 압력이 다시
유효하면 별도의 service true 호출 없이 자동 arm된다. `/walker/set_enabled false`
는 필요할 때 자동 arm을 수동으로 막는 별도 inhibit다.

MPU pitch가 5도 경계를 0.5초 유지하면 경사로 확정한다. 내리막은 속도 배율을
0.60으로 낮추고 오르막은 1.25로 높이되, 거친 노면 등의 감속 배율을 오르막
보정이 상쇄하지 못한다. MPU 오류는 경사 배율을 1.0으로 되돌리고 진단에
표시하며 TOF hazard 정지를 우회하지 않는다.

GPS와 횡단보도 기능은 기본적으로 관찰 전용이다. 지도와 API가 없어도 진단을
발행하지만 motion command 경로에는 참여하지 않는다. Foxglove도 기본 비활성이며
활성화할 때 publish/service/parameter 기능을 제한한 읽기 전용 설정을 사용한다.

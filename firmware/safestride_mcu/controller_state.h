// Drive 보드 상태 코드. 숫자는 통신 상태값에 쓰이므로 임의로 바꾸지 않는다.
// BOOT: 초기화, DISARMED: 구동 해제, ARMED: 구동 허용, SAFE_STOP: 안전 정지,
// ESTOP: 비상정지, FAULT: 고장. ARMED 상태에서도 명시적 BRAKE로 출력은 정지할 수 있다.

#pragma once

#include <Arduino.h>

// Kept in a header so Arduino's generated sketch prototypes can see the type.
enum class ControllerState : uint8_t {
  BOOT = 0U,
  DISARMED = 1U,
  ARMED = 2U,
  SAFE_STOP = 3U,
  ESTOP = 4U,
  FAULT = 5U,
};

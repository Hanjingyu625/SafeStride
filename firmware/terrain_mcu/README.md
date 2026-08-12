# Terrain Uno firmware

This controller owns TOF-10120, MPU-9250, BNO055 and the leg actuator. The
current sketch is intentionally non-driving: `LEG_OUTPUT_ENABLED` is false.

The TOF-10120 is read over I2C at address `0x52` every 50 ms. Its distance is
filtered with an EMA and compared with a slower adaptive reference. A 60 mm
error is a step candidate; an error above 60 mm together with a filtered rise
above 10 mm for four consecutive frames is a confirmed step. The red result is
held for one second. D8/D9/D10 show normal/candidate/step, and an invalid I2C
sample is shown as red.

Final states are `STOWED`, `DEPLOYING`, `DEPLOYED`, `RETRACTING`, `SAFE_STOP`
and `FAULT`. Motion requires a fresh host command, both limits, valid attitude,
low wheel speed and a local deadline.

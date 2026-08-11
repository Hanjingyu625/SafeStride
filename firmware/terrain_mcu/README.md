# Terrain Uno firmware

This controller owns TOF-10120, MPU-9250, BNO055 and the leg actuator. The
current sketch is intentionally non-driving: `LEG_OUTPUT_ENABLED` is false and
no sensor library is selected until exact module variants and pins are known.

Final states are `STOWED`, `DEPLOYING`, `DEPLOYED`, `RETRACTING`, `SAFE_STOP`
and `FAULT`. Motion requires a fresh host command, both limits, valid attitude,
low wheel speed and a local deadline.

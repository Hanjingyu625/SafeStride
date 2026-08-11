# Hardware integration register

All pins, voltage levels and addresses are placeholders until checked against
the exact modules. Do not connect 5 V Uno signals to non-tolerant 3.3 V modules.

| Device | Owner | Proposed bus | Notes |
|---|---|---|---|
| Wheel motors x2 | Drive Uno | PWM/direction/enable | One rated driver per motor |
| Wheel encoders x2 | Drive Uno | interrupt GPIO | Existing code assumes quadrature |
| Round pressure sensors x2 | Drive Uno | analog | Fixed resistors and per-handle calibration required |
| E-stop | Drive Uno + hardware chain | normally closed | A broken wire must stop motion |
| TOF-10120 | Terrain Uno | verify I2C/UART variant | Measure mounting geometry |
| MPU-9250 | Terrain Uno | I2C, usually 0x68/0x69 | Verify level shifting |
| BNO055 | Terrain Uno | I2C, usually 0x28/0x29 | Publish calibration status |
| Leg motor | Terrain Uno | PWM/direction/enable | Needs a third driver, current protection and two limits |
| BE-220 GPS | Raspberry Pi | dedicated UART/USB | Keep separate from Arduino device paths |
| Camera | Raspberry Pi | CSI or USB | Calibrate after rigid mounting |

## Required before assigning pins

- Exact model and datasheet for all three motor drivers.
- Motor voltage, stall current and encoder specification.
- Leg travel, required torque, direction and holding behavior.
- TOF-10120 electrical-interface variant and field of view.
- Pressure sensor resistance range and mechanical preload.
- Raspberry Pi, OS and camera model.

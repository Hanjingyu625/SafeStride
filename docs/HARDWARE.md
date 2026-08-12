# Hardware integration register

All pins, voltage levels and addresses are placeholders until checked against
the exact modules. Do not connect 5 V Uno signals to non-tolerant 3.3 V modules.

| Device | Owner | Proposed bus | Notes |
|---|---|---|---|
| Wheel motors x2 | Drive Uno | PWM/IN1/IN2 | One SZH-GNP521 per motor |
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

## Integrated Arduino pin map

The Drive Uno uses D5/D6/D8 for the left driver's PWM/IN1/IN2 and
D9/D10/D12 for the right driver's PWM/IN1/IN2. A0/A1 read the two FSR voltage
dividers, A2 reads the normally-closed E-stop, and A3/A4/A5 drive the pressure
state LEDs. D2/D3 remain the encoder interrupt pins, with D4/D7 as encoder B.
Connect both driver `COM` terminals to Arduino GND so the 5 V control signals
share a reference. The driver `5VO` terminals are outputs and remain
unconnected; they are not Arduino 5 V inputs. Add a 10 kOhm pull-down from each
driver PWM input to COM so the drivers remain stopped while the Uno resets.

The Terrain Uno uses I2C A4/A5 for the TOF-10120 and D8/D9/D10 for its
green/yellow/red LEDs. The two Unos have separate pin maps.

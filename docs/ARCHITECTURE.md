# SafeStride system architecture

SafeStride uses three independently supervised computers. Linux may request
motion, but it must never be the only layer capable of stopping an actuator.

## Responsibility split

| Controller | Connected hardware | Responsibility |
|---|---|---|
| Raspberry Pi | Camera, BE-220 GPS, two USB serial links | ROS 2, YOLO, crosswalk logic, logging and high-level requests |
| Drive Uno | Two wheel encoders, two handle pressure sensors, E-stop, two wheel drivers | Final wheel enable, velocity control and wheel watchdog |
| Terrain Uno | TOF-10120, MPU-9250, BNO055, leg limits, leg motor driver | Step detection, redundant attitude checks and leg state machine |

The pressure sensors form a two-channel handle-presence/dead-man input, not a
calibrated weight measurement. Losing either hand requests a safe stop unless a
later validated operating mode explicitly permits one-hand use.

## Data flow

```text
BE-220 GPS -> navigation/crosswalk --+
camera -> YOLO surface estimate -----+-> safety supervisor -> Drive Uno
Terrain Uno -> step/attitude/state --+          |
                                                +-> terrain coordinator -> Terrain Uno
```

YOLO output is advisory. It may reduce permitted speed but may not write PWM,
enable motors or bypass stale sensor checks. Unknown, stale or low-confidence
classification uses the conservative speed limit.

TOF may propose a step. Leg deployment additionally requires low wheel speed,
valid attitude, both hands present, fresh Pi permission, valid limit switches
and an MCU-local timeout. Any failed condition stops wheel and leg motion.

## ROS packages

- `safestride_interfaces`: shared message contracts.
- `safestride_bridge`: fail-safe Drive Uno serial bridge.
- `safestride_control`: ROS-side wheel command safety supervisor.
- `safestride_sensors`: BE-220 NMEA and sensor adapters.
- `safestride_perception`: YOLO adapter and surface speed policy.
- `safestride_terrain`: terrain and leg coordination policy.
- `safestride_bringup`: launch and deployment configuration.
- `safestride_description`: geometry and sensor frames.

## Non-negotiable safety boundaries

1. Motor PWM/enable inputs need external bias that disables drivers during reset.
2. E-stop must interrupt driver enable electrically, not only through software.
3. The leg requires retracted and deployed limit switches.
4. Both Unos invalidate their sessions after a watchdog timeout.
5. Deployment is forbidden above the configured wheel-speed threshold.
6. Initial builds keep all motor output disabled in configuration.

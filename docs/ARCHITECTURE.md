# SafeStride system architecture

SafeStride uses three independently supervised computers. Linux may request
motion, but it must never be the only layer capable of stopping an actuator.

## Responsibility split

| Controller | Connected hardware | Responsibility |
|---|---|---|
| Raspberry Pi | Camera, two USB serial links | ROS 2, road-surface classification, crosswalk logic, logging and high-level requests |
| Drive Uno | Reserved wheel-encoder inputs, two handle pressure sensors, one shared motor driver | Final common wheel enable, velocity control and encoder-feedback gate; E-stop input is reserved but not implemented |
| Terrain Uno | TOF-10120, BE-220 GPS, future IMUs/leg hardware | TOF/GPS acquisition; future step and leg state machine |

The pressure sensors form a two-channel handle-presence/dead-man input, not a
calibrated weight measurement. Losing either hand requests a safe stop unless a
later validated operating mode explicitly permits one-hand use.

## Data flow

```text
Terrain Uno GPS -> navigation/crosswalk ---+
Pi camera -> TorchScript surface estimate -+-> safety supervisor -> Drive Uno
Terrain Uno -> step/attitude/state --------+          |
                                                      +-> terrain coordinator -> Terrain Uno
```

Surface-classifier output is advisory. It applies a bounded 0.0–1.25 speed
scale but may not write PWM, enable motors or bypass stale sensor checks. The
result is clamped again by the absolute ROS speed limits. Unknown, stale or
low-confidence classification stops motion when perception is enabled.

TOF may propose a step. Leg deployment additionally requires low wheel speed,
valid attitude, both hands present, fresh Pi permission, valid limit switches
and an MCU-local timeout. Any failed condition stops wheel and leg motion.

## ROS packages

- `safestride_interfaces`: shared message contracts.
- `safestride_bridge`: fail-safe Drive Uno serial bridge.
- `safestride_control`: ROS-side wheel command safety supervisor.
- `safestride_sensors`: BE-220 NMEA and sensor adapters.
- `safestride_navigation`: crosswalk geometry, V2X timing and automatic crossing policy.
- `safestride_perception`: Pi camera classifier and surface speed policy.
- `safestride_terrain`: terrain and leg coordination policy.
- `safestride_bringup`: launch and deployment configuration.
- `safestride_description`: geometry and sensor frames.

## Non-negotiable safety boundaries

1. Motor PWM/enable inputs need external bias that disables drivers during reset.
2. Before an E-stop is implemented, testing requires a separate physical motor-power disconnect; a future E-stop must interrupt driver enable electrically, not only through software.
3. The leg requires retracted and deployed limit switches.
4. Both Unos invalidate their sessions after a watchdog timeout.
5. Deployment is forbidden above the configured wheel-speed threshold.
6. Initial builds keep all motor output disabled in configuration.

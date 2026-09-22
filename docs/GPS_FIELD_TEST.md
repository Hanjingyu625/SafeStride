# GPS and crosswalk field-test changes

These changes cover `safestride_navigation`, its YAML defaults and LCD Lua scripts. They do not
change Arduino firmware, ToF policy, motor PWM, or the display wire format.
The working branch is `codex/pdj1-slope-control`; no remote branch is updated
automatically. A running Pi may have different settings or older code.

## What the existing system measures

- Crosswalk latitude/longitude is its centre, not an entry endpoint. The
  converter derives a centre and axis from the source polygon. Runtime uses
  an oriented rectangle with `length_m`, `width_m`, and `axis_bearing_deg`.
  `edge_distance_m` is distance to this rectangle, including its sides.
- Progress is the projection along the crossing axis, with zero at the
  estimated near endpoint and `length_m` at the far endpoint. It is not a
  surveyed observation of either curb. Bad geometry and GPS bias still matter.
- GPS course is movement direction, not the direction the connector points.
  Current software uses NMEA course and displacement, with wheel-motion gating.
  A stationary turn cannot be detected from this information alone.
- The speed profile is the rolling median of accepted motion samples,
  not an average including waiting time. Zero and samples below 0.12 m/s are
  excluded. Waiting, entry-allowed, and urgent states do not update the profile.
  Samples are collected at controller tick rate, not independently per Hall pulse.
  The old 0.30 m/s lower clamp is removed to avoid overestimating slow walkers.
  Speeds below the accepted range still need separate sensor/profile calibration.
  Without saved or new samples, the default estimate is now 1.0 m/s (previously
  0.5). This is an assumption, not feedback or proof of user capability. Learned
  speeds retain the 1.0 m/s upper cap. Existing saved samples are preserved.

## What changed

- Candidates must agree with both the direction toward the crosswalk centre
  and the crossing axis. The heading remains usable until its existing timeout
  even immediately after stopping. The 80 m search and 60 degree tolerance remain.
- Isolated GPS jumps are rejected without averaging normal fixes. The gate
  allows 5 m of position noise plus 3 m/s of movement (elapsed time capped at
  5 s). Three nearby fixes reacquire a distant cluster. This rejects outliers;
  it cannot remove persistent GNSS bias or improve the receiver update rate.
- Control position is held while fresh wheel odometry confirms standstill,
  after the existing 3.5 s motion hold expires. Movement releases it immediately.
  GPS gaps/invalid fixes release the hold. Raw `/gps/fix` is unchanged. Hall
  failure can defeat standstill detection; this does not replace localization.
- Entering needs along-axis progress inside the crossing and lateral error
  within 3 m of the rectangle. When odometry exists, actual wheel displacement
  is also required. Moving along the waiting curb alone no longer starts crossing.
- GPS progress is capped by wheel progress plus a 3 m tolerance. Completion
  requires the far-end GPS check, wheel-distance check when available, lateral
  corridor check, and existing clearance/hold timers. GPS-only operation still
  depends on GPS accuracy. Wheel distance is not heading-aware and can overcount
  detours; this is not a full localization/filtering system.
- Loss of GPS during crossing preserves an urgent crossing state rather than
  resetting to IDLE. Existing invalid-GPS command gating remains in force.
- Signal loss, missing ETA, and genuinely short signal time have different
  `reason` strings. Existing state numbers and display protocol are preserved.
- Signal seconds decrease between API responses, including request latency.
  Duplicate/out-of-order source timestamps cannot renew cache freshness.
  Source UTC timestamps (epoch milliseconds) must also be fresh; Pi time must
  be synchronized. Phase and countdown timestamps must agree within 3 s.
- Automatic intersection matching is withheld when the nearest two candidates
  differ by less than `intersection_ambiguity_margin_m` (default 10 m).
  This is an ambiguity check, not proof that the nearest controller serves the curb.

## Signal mapping still needs field verification

Runtime now fetches phase and countdown independently and uses the SAME
direction's `PdsgStatNm` and `PdsgRmdrCs`, not the opposing head's minimum.
`stop-And-Remain` produces WAIT with valid signal and zero usable green time.
`protected-Movement-Allowed` additionally requires a fresh matching countdown.
Missing/unsupported phase, timeout or mismatched data produces NO SIGNAL DATA,
never entry permission. The legacy opposing-min parser remains only for compatibility.
The inferred signal direction remains
crossing-axis + 90 degrees unless overridden. Geometry alone cannot establish
which signal head serves a particular crossing. Verify the physical head and
crosswalk mapping before treating the display as guidance. This is not a
certified road-crossing safety system.

The new `signal_phase_url` parameter defaults to the official
`v2xSignalPhaseInformation/1.0` endpoint. Confirm API authorization for this
service separately from timing. The official current-phase service also exposes
`v2xSignalPhaseCurrentInfo/1.0`; its key parameter is `apikey` rather than the
legacy `apiKey` and must be handled if switching endpoints.
References: https://t-data.seoul.go.kr/category/dataviewopenapi.do?data_id=10119
and https://t-data.seoul.go.kr/dataprovide/trafficdataviewopenapi.do?data_id=10380.

For a field-verified mapping, the source crosswalk JSON record can contain:

```json
{
  "latitude": 37.5,
  "longitude": 127.0,
  "length_m": 15.0,
  "width_m": 4.0,
  "axis_bearing_deg": 0.0,
  "intersection_id": "VERIFIED_ID",
  "signal_direction": "nt"
}
```

This is a schema example, not a real mapped crossing. Use measured/source
geometry and a verified ID/direction. Preserve this data when regenerating the
map. Allowed direction codes: nt, ne, et, se, st, sw, wt, nw.

## Why 26 seconds can be insufficient

Entry time is now `full length / profile speed + 1 s reaction + 2 s margin`.
Both margins are configurable (`reaction_time_s`, `entry_safety_margin_s`).
A 10 m crossing at 1.0 m/s needs 13 s, so both 26 s and 18 s allow entry with
a verified green phase. At a learned 0.35 m/s, it needs 31.6 s: neither allows
entry. Red always means wait before entry. A 16.48 m crossing at 0.35 m/s needs
about 50.1 s. During crossing, compare remaining distance / speed
plus 2 s with the countdown: 8.5 m at 0.5 m/s requires 19 s, so 26 s passes
and 18 s does not. Changing the threshold to make the screen look normal would
hide this difference. Check Hall pulses, wheel radius and measured travel first.

## Build and observe on the Pi

After transferring/committing these changes and pulling the intended branch:

```bash
cd ~/SafeStride
source /opt/ros/jazzy/setup.bash
colcon build --packages-select safestride_navigation
source install/setup.bash
```

Restart the existing bringup once with its existing environment/settings.
Do not start a second serial bridge. Verify deployed motor-output settings:

```bash
ros2 param get /crosswalk_controller motion_output_enabled
ros2 topic echo /crosswalk/status safestride_interfaces/msg/CrosswalkStatus
```

Record one complete approach/wait/cross/exit in another correctly sourced Pi
terminal (same ROS_DOMAIN_ID as bringup):

```bash
ros2 bag record /gps/fix /gps/speed /gps/course /odom /crosswalk/status /diagnostics /walker/status /cmd_vel /cmd_vel_safe
```

Diagnostics now include `signal_reason`, `signal_record_timestamp`,
`signal_cache_age_s`, `profile_speed_mps`, `profile_sample_count`,
`crosswalk_length_m`, `crosswalk_lateral_error_m`, and `gps_fix_gate`.
Existing diagnostics include intersection selection source and candidate angle.
Observe waiting speed stability, normal CROSSING with adequate time, no entry
while walking beside the curb, and completion only after reaching the far side.
Reload `display/ezhmi/visualtft/main.lua` into the LCD project (the standalone
`display/ezhmi/safestride.lua` copy is identical). Pulling/building the Pi alone
does not change LCD text. Display states are CAN CROSS, WAIT, CROSSING, CAUTION,
NO SIGNAL DATA and CROSSING COMPLETE. Already crossing with insufficient time
shows CAUTION, not an instruction to stop in the road. Missing signal shows
NO SIGNAL DATA even during crossing. The LCD project's initial text is
Location: N/A. With live HMI v3 data it changes to the romanized intersection
name without a Location: prefix, or CROSSWALK NEARBY when a crosswalk has no
name. If the physical LCD still shows GPS: FIX OK / GPS: NO FIX, it has an older
screen package and must be recompiled and downloaded from the VisualTFT project.
The current 26-word wire format transmits up to 20 ASCII location bytes, but it
does not transmit latitude/longitude to the LCD.

## Local verification

```bash
PYTHONPATH=src/safestride_navigation python3 -m unittest discover -s src/safestride_navigation/test
```

Unit tests exercise waiting-speed exclusion, slow walkers, perpendicular
candidates, ambiguous intersections, isolated GPS jumps/reacquisition, entry
outside the roadway, GPS jumps at the far curb, signal age and duplicate
timestamps, normal/urgent recovery, and the existing crossing lifecycle.
They do not replace a ROS/Pi run or validate the five field-tested crossings.

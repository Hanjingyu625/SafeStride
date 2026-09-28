# GPS crosswalk assistance

The standalone `smart_crosswalk_controller_v6.py` logic has been split into
testable ROS 2 components. The Raspberry Pi `gps_node` reads the BE-220 directly
from `/dev/serial0` or `/dev/ttyS0` and publishes `/gps/fix`, filtered
`/gps/speed`, diagnostic `/gps/speed_raw`, and motion-gated `/gps/course`.
Terrain Uno does not relay GPS data. The crosswalk controller is monitor-only by
default and therefore does not publish `/cmd_vel` unless explicitly enabled.

The GPS speed filter requires a fresh GGA quality report, at least five
satellites, HDOP no worse than 5.0, and spatially consistent movement over a
rolling window. Raw RMC speed alone never proves movement. Wheel-derived
`/odom` is authoritative whenever it is fresh, including its zero-speed state;
the filtered GPS speed is only a fallback. This prevents stationary GNSS drift
from changing the user speed profile. Very slow walking is retained by Hall
odometry rather than guessed from low-cost GNSS noise.

## Prepare crosswalk data

The supplied A004 shapefile is external data and is intentionally ignored by
Git. Copy its `.shp`, `.dbf`, `.shx`, `.prj` and `.cpg` files under
`data/external/crosswalk_shp/`, then run:

```bash
sudo apt install python3-shapefile python3-pyproj
python3 tools/convert_crosswalk_shp.py \
  data/external/crosswalk_shp/A004_A.shp \
  --output data/generated/standard_crosswalks.json
```

`scripts/run.sh` uses the tracked
`raspberry_pi/standard_crosswalks.json` by default. Override it with
`SAFESTRIDE_CROSSWALK_FILE=/absolute/path/crosswalks.json`. The controller
builds a spatial index once, then searches only nearby records rather than
scanning the complete map at 5 Hz.

The tracked `raspberry_pi/v2x_intersections.json` is the official Seoul T-Data
intersection MAP CSV normalized for offline startup. Override it with
`SAFESTRIDE_INTERSECTION_MAP_FILE=/absolute/path/intersections.json`. When the
API is available, the controller refreshes this seed in the background and
caches the newer result without blocking signal timing requests.

## Heading and candidate selection (2026-09-28)

While motion is confirmed, fresh GPS RMC course or at least 2 m of GPS
position movement supplies the initial north reference. `/terrain/imu` then
propagates that bearing through turns, including pivots without translation.
The IMU quaternion supplies only gravity roll/pitch, never absolute yaw.
The node computes yaw rate from the sensor Y/Z rates and roll/pitch, then
converts ROS counterclockwise yaw to clockwise compass bearing. This assumes
the documented upright `imu_link` mounting; `heading_gyro_sign` defaults to
+1 and permits -1 for an inverted yaw sign. It is not an arbitrary mounting
transform. Verify that a clockwise 90-degree pivot increases bearing by 90
degrees before relying on the field results.

GPS corrections use 20% of the shortest angular difference. During turns
above 3 deg/s and for 1 second afterward, GPS course corrections are suppressed
to avoid restoring the previous trajectory. Quiet wheel-confirmed standstill
and absolute yaw rate below 0.5 deg/s for 2 seconds permit slow bias learning
(time constant 10 seconds). A slow pivot can still resemble bias; this is a
bounded estimate, not a compass measurement.

IMU stamps, frame, validity, quaternion norm and finite values are checked;
roll or pitch beyond 45 degrees invalidates this heading path. A gap exceeding
0.35 seconds or non-increasing IMU timestamps discards the integrated heading.
A new GPS observation is needed to reanchor. GPS-only heading expires after
5 seconds. With continuous IMU, heading expires 20 seconds after the last
accepted GPS anchor. Confidence is a heuristic decreasing linearly from 1
to 0 over that interval, not a calibrated probability. It must be at least
0.5 to confirm a crosswalk. With defaults, gyro-only confirmation is therefore
possible up to 10 seconds after the GPS anchor; GPS-only confirmation up to
2.5 seconds. Replayed GPS observations never refresh the anchor age.

The spatial index still searches within 80 m of a crosswalk edge, but the ROS
node ranks all nearby candidates instead of rejecting those over 60 degrees
from the heading. Ranking uses distance to the nearest end of the crossing,
heading agreement with both its centre and axis, and recent change in entrance
distance. The score is entrance distance plus confidence-weighted angular
penalties (12 m at 90-degree axis error, 12 m at 180-degree centre bearing
error), minus approach rate clipped to +/-2 as a score bonus. These are
ranking weights, not GPS error estimates. A side-on crossing remains visible
as a candidate before a turn.

Confirmation requires heading confidence >=0.5, axis error <=60 degrees,
centre bearing error <=60 degrees (waived within 1 m of the polygon), and at
least a 3-point lead over the next candidate for 2 continuous seconds. Gaps
between selection updates over 1 second restart confirmation. Ties or missing
heading keep candidates visible but do not grant entry. `/crosswalk/guidance`
reports `방향 확인 중`; the existing LCD continues to use the seven-state status.

At 50 m the policy approaches, at 18 m it may provisionally lock a confirmed
candidate, and at 7 m it checks the corridor and signal window. Before entry,
a candidate/direction change or lost confirmation clears the old lock,
intersection ID, progress history and entry grant **before** signal lookup.
The new intersection and pedestrian direction are evaluated again; an old
intersection response cannot authorize the new one. Uncertain selection within
7 m requests WAIT_AT_CURB (zero policy target). Confirmation can restore the
usual signal checks. Once crossing has been detected, the selected crossing
remains locked despite changing candidates or loss of heading. Existing
crossing-completion and urgent-assistance rules remain in effect.

Diagnostics expose `heading_source`, `heading_confidence`,
`selection_confirmed`, `selection_margin_m`, `selection_candidate_count`, and
`entrance_distance_m` alongside the existing GPS and signal fields. These
changes improve selection logic but do not remove GPS/map position error or
supply an initial heading while stationary. Runtime defaults keep motor output
disabled. Field validation still needs actual sensor logs.

## Configure signal timing

`scripts/run.sh` first uses `/etc/safestride/signal_api_key.txt` when it exists,
then falls back to `raspberry_pi/api_key.txt` for this deployed prototype:

```bash
sudo install -d -m 750 /etc/safestride
sudo install -o "$USER" -g "$(id -gn)" -m 600 /path/to/new-key.txt \
  /etc/safestride/signal_api_key.txt
```

With a key present, the ROS node asynchronously downloads the V2X intersection
map, caches it under `~/.cache/safestride/`, matches the selected crosswalk to
an intersection within 120 m, and then requests its pedestrian signal timing.
It first tries Seoul T-Data's current combined phase/countdown endpoint. If
that service is unavailable to the configured key, it falls back to the two
previously configured phase and timing endpoints. Both `protected` and
`permissive` pedestrian green require a fresh matching countdown before entry
can be allowed; a countdown by itself is never treated as a green signal.
`SAFESTRIDE_INTERSECTION_ID` is only a bench fallback when no API key is
configured. A configured key always enables live matching and never falls back
to a stale fixed ID. With no valid ID, API key, network, or fresh signal value,
the policy fails closed at the curb and reports the reason in `/diagnostics`.

Each selection change is logged as GPS coordinates, crosswalk index and
distance, intersection ID and name, and the ID source. The same fields are in
the `SafeStride/Crosswalk Controller` diagnostic, so a stale fixed ID cannot be
mistaken for a live GPS match.

## Start in monitor-only mode

Keep `motion_output_enabled: false` for GPS walks and inspect:

```bash
export SAFESTRIDE_ENABLE_CROSSWALK=true
bash scripts/run.sh
ros2 topic echo /crosswalk/status
ros2 topic echo /gps/fix
ros2 topic echo /gps/speed
ros2 topic echo /gps/speed_raw
ros2 topic echo /gps/course
ros2 topic echo /diagnostics --field status
```

Monitor-only mode publishes status without becoming a `/cmd_vel` publisher.
The configured cruise target is 1.0 m/s. With a 1.0 m/s walking profile and
feedback, the crosswalk target is 1.0 m/s during normal crossing and 1.1 m/s
during caution (the existing +0.10 m/s assistance). Its configurable
`maximum_assist_speed_mps` is 1.15 m/s, matching the supervisor forward limit
and MCU target limit of 10000 mrad/s at the configured 0.115 m wheel radius.
Keep these limits aligned when changing the drive configuration. The walking
profile still learns actual moving speed for crossing-time estimates; raising
the command ceiling does not assume a slow user can walk faster. Approach and
exit targets remain capped at 0.50 m/s, and waiting requests zero speed.
In monitor-only mode these are status targets, not motor commands. When motion
output is enabled, supervisor slope scaling, acceleration limits, grip checks,
and MCU Hall-feedback/PWM limits still apply; a requested speed is not a
guarantee of measured speed. The old standalone v6 script is not this ROS path.

Diagnostics include coordinates, heading source, candidate bearing, crossing
direction, matched `itstId`, and distance to the intersection. Verify every
state transition from recorded logs before enabling motion. If crosswalk motion
output is enabled, disable `cruise_command` so two nodes do not publish competing
commands. A human trial must not be the first powered test.

The automatic sequence is:

```text
IDLE -> APPROACHING -> WAIT_AT_CURB or ENTRY_ALLOWED
     -> CROSSING or CROSSING_URGENT -> EXITING -> IDLE
```

`/crosswalk/status.state` exposes seven internal phases numbered 0 through 6,
not four display instructions. The main display decisions are CAN CROSS (fresh
green with enough time), CAUTION (already crossing and time is tight), WAIT
(not enough time or red), and NO SIGNAL DATA (signal unavailable, including
during an internal urgent crossing). Normal
CROSSING and CROSSING COMPLETE are separate progress indications. A
photo of a green lamp does not make `signal_valid` true: the selected V2X
intersection ID, pedestrian direction, source timestamp, and countdown must
all match. The `SafeStride/Crosswalk Controller` `/diagnostics` entry includes
`signal_reason`, `signal_direction`, `signal_phase_value`,
`signal_raw_countdown`, `wheel_odometry_fresh`, and crosswalk distance for
field diagnosis. Inspect it alongside `/crosswalk/status`; keep motion output
disabled until its selected signal head matches the physical crossing.
`entry_allowed` is true only in `ENTRY_ALLOWED` (state 3); it never means
"continue crossing" in states 4-6. Crossing assistance can remain active after
entry even when a new entry would not be safe.

If entry is detected while waiting, the policy changes to
`CROSSING_URGENT`; it does not command a stop after the user is already in the
roadway. Signal loss before entry stops at the curb, while signal loss during a
crossing requests continued assistance and an urgent status. Local hardware
safety can always override this request.
# Human-readable guidance topic

`ros2 topic echo /crosswalk/guidance std_msgs/msg/String`

This topic publishes JSON with Korean `state` and `reason` strings on each
status update. Guidance states are CAN CROSS, CAUTION, WAIT, NO SIGNAL (shown
in Korean), plus NO GUIDANCE for no selected crossing, missing GPS before
entry, or completed crossing. CAUTION distinguishes short signal time from
unknown position, ETA, or completion; only short time advises prompt crossing.
Signal loss during crossing does not instruct stopping in the road.

The existing `/crosswalk/status` seven-state interface remains compatible with
motor control and LCD firmware. This additional topic does not change the LCD
layout or motor behavior. Read it after rebuilding and restarting navigation.

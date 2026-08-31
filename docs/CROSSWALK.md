# GPS crosswalk assistance

The standalone `smart_crosswalk_controller_v6.py` logic has been split into
testable ROS 2 components. The Raspberry Pi `gps_node` reads the BE-220 directly
from `/dev/ttyS0` and publishes `/gps/fix`, `/gps/speed` and `/gps/course`.
Terrain Uno does not relay GPS data. The crosswalk controller is monitor-only by
default and therefore does not publish `/cmd_vel` unless explicitly enabled.

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

When RMC course is fresh, or the GPS position has moved at least 2 m, candidates
more than 60 degrees away from the travel direction are rejected. Before a
heading is available, the nearest polygon is used. GPS course is direction of
travel, not a compass heading, so it is intentionally unavailable while the
walker is stationary.

## Configure signal timing

Do not commit the Seoul V2X API key. Store only the key in a user-readable file
such as `/etc/safestride/signal_api_key.txt`:

```bash
sudo install -d -m 750 /etc/safestride
sudo install -m 600 /path/to/new-key.txt \
  /etc/safestride/signal_api_key.txt
```

With a key present, the ROS node asynchronously downloads the V2X intersection
map, matches the selected crosswalk to an intersection within 120 m, and then
requests its pedestrian signal timing. A tested fixed ID can override matching
with `SAFESTRIDE_INTERSECTION_ID=1678`. With no valid ID, API key, network, or
fresh signal value, the policy fails closed at the curb and reports the reason
in `/diagnostics`.

## Start in monitor-only mode

Keep `motion_output_enabled: false` for GPS walks and inspect:

```bash
export SAFESTRIDE_ENABLE_CROSSWALK=true
bash scripts/run.sh
ros2 topic echo /crosswalk/status
ros2 topic echo /gps/fix
ros2 topic echo /gps/course
ros2 topic echo /diagnostics --field status
```

Monitor-only mode publishes status without becoming a `/cmd_vel` publisher.
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

If entry is detected while waiting, the policy changes to
`CROSSING_URGENT`; it does not command a stop after the user is already in the
roadway. Signal loss before entry stops at the curb, while signal loss during a
crossing requests continued assistance and an urgent status. Local hardware
safety can always override this request.

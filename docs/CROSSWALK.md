# GPS crosswalk assistance

The standalone `smart_crosswalk_controller_v6.py` logic has been split into
testable ROS 2 components. By default Terrain Uno publishes BE-220 fixes and
speed through `terrain_bridge`, while `crosswalk_controller` publishes a
high-level `/cmd_vel`. Every command still
passes through `safety_supervisor`, the Drive serial bridge and the Drive Uno's
dead-man, E-stop, watchdog and fault checks.

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

Set `crosswalk_controller.ros__parameters.crosswalk_file` in the deployment
YAML to the resulting absolute path.

## Configure signal timing

Do not commit the Seoul V2X API key. Store only the key in a user-readable file
such as `/etc/safestride/signal_api_key.txt`, and set `api_key_file` in the
deployment YAML.

The supplied v6 folder refers to `nearest_map.py`, but that file is not in the
folder. Until an intersection-map source is added, set `intersection_id` to the
tested `itstId` for the trial location. Alternatively, enrich each converted
crosswalk record with an `itstId`, `itst_id` or `intersection_id` field. With no
valid ID, API key or fresh signal value, the policy fails closed at the curb.

## Start in monitor-only mode

Keep `motion_output_enabled: false` for GPS walks and inspect:

```bash
export SAFESTRIDE_ENABLE_CROSSWALK=true
bash scripts/run.sh
ros2 topic echo /crosswalk/status
ros2 topic echo /diagnostics
```

Monitor-only mode publishes zero velocity while reporting the command the v6
state machine selected. Verify the crosswalk axis, signal direction, `itstId`,
GPS accuracy and every state transition from recorded logs before enabling
motion. A human trial must not be the first powered test.

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

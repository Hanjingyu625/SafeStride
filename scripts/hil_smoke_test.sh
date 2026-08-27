#!/usr/bin/env bash
set -euo pipefail

workspace="${SAFESTRIDE_WORKSPACE:-/opt/safestride}"
config="${SAFESTRIDE_CONFIG:-${workspace}/config/raspberry_pi.yaml}"

for device in /dev/safestride-drive /dev/safestride-terrain; do
  if [[ ! -e "${device}" ]]; then
    echo "HIL FAIL: missing ${device}" >&2
    exit 1
  fi
done

source /opt/ros/jazzy/setup.bash
source "${workspace}/install/setup.bash"

ros2 launch safestride_bringup safestride.launch.py \
  config_file:="${config}" \
  enable_cruise:=false \
  enable_perception:=false \
  enable_gps:=true \
  enable_crosswalk:=true > /tmp/safestride-hil.log 2>&1 &
launch_pid=$!
trap 'kill "${launch_pid}" 2>/dev/null || true' EXIT

for topic in /walker/status /wheel/hall /handle/pressure \
             /terrain/status /terrain/imu /gps/fix /diagnostics; do
  if ! timeout 20 ros2 topic echo --once "${topic}" >/dev/null; then
    echo "HIL FAIL: no message on ${topic}" >&2
    exit 1
  fi
done

# The smoke test never arms the motor. Explicitly request disabled state and
# confirm that the service is reachable before reporting success.
timeout 10 ros2 service call \
  /walker/set_enabled std_srvs/srv/SetBool "{data: false}" >/dev/null

echo "HIL PASS: both Uno links and required ROS sensor topics are active"
echo "Inspect /tmp/safestride-hil.log and move an object under the TOF to"
echo "confirm TOF_RAISED/TOF_DROP while wheels remain lifted."

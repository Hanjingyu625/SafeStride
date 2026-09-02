#!/usr/bin/env bash
set -euo pipefail

workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

usage() {
  cat <<'EOF'
Usage: bash scripts/test_drive_pi.sh --enable-motor [duration_seconds]

Requires scripts/run.sh to be running in another terminal. The powered wheels
must be lifted and a physical power cutoff must be within reach.
EOF
}

if [[ "${1:-}" != "--enable-motor" ]]; then
  usage >&2
  exit 2
fi

duration="${2:-30}"
if [[ ! "${duration}" =~ ^[0-9]+$ ]] ||
    (( duration < 1 || duration > 120 )); then
  echo "duration_seconds must be an integer from 1 to 120" >&2
  exit 2
fi

set +u
source /opt/ros/jazzy/setup.bash
source "${workspace}/install/setup.bash"
set -u

unset ROS_LOCALHOST_ONLY
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-SUBNET}"

command_pid=""
cleanup() {
  set +e
  if [[ -n "${command_pid}" ]]; then
    kill "${command_pid}" >/dev/null 2>&1
    wait "${command_pid}" >/dev/null 2>&1
  fi
  timeout 3s ros2 topic pub --once /cmd_vel \
    geometry_msgs/msg/TwistStamped \
    "{twist: {linear: {x: 0.0}, angular: {z: 0.0}}}" \
    >/dev/null 2>&1
  timeout 3s ros2 service call /walker/set_enabled \
    std_srvs/srv/SetBool "{data: false}" >/dev/null 2>&1
}
trap cleanup EXIT INT TERM

if [[ ! -e /dev/safestride-drive ]]; then
  echo "/dev/safestride-drive is missing" >&2
  exit 1
fi

status="$(timeout 5s ros2 topic echo /walker/status --once)" || {
  echo "No fresh /walker/status received" >&2
  exit 1
}
if ! grep -q 'link_ok: true' <<<"${status}"; then
  echo "Drive link is not ready:" >&2
  echo "${status}" >&2
  exit 1
fi
if grep -Eq 'fault_bits: [1-9][0-9]*' <<<"${status}"; then
  echo "Drive reports a fault:" >&2
  echo "${status}" >&2
  exit 1
fi

echo "Publishing a supervised 0.08 m/s command for ${duration}s."
echo "Hold both pressure sensors; no set_enabled true call is required."
echo "Releasing either side requests the 0.6 s dead-man stop ramp."
echo "The left A3 WSH135 Hall sensor closes the shared motor speed loop."
timeout --signal=INT "${duration}s" ros2 topic pub --rate 20 /cmd_vel \
  geometry_msgs/msg/TwistStamped \
  "{twist: {linear: {x: 0.08}, angular: {z: 0.0}}}" \
  >/dev/null &
command_pid=$!
set +e
wait "${command_pid}"
publisher_status=$?
set -e
command_pid=""
if (( publisher_status != 0 && publisher_status != 124 )); then
  echo "Command publisher failed with status ${publisher_status}" >&2
  exit "${publisher_status}"
fi
echo "Test complete; publishing zero and setting the manual inhibit."

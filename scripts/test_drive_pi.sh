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

publisher_pid=""
cleanup() {
  set +e
  timeout 3s ros2 service call /walker/set_enabled std_srvs/srv/SetBool \
    "{data: false}" >/dev/null 2>&1
  if [[ -n "${publisher_pid}" ]]; then
    kill "${publisher_pid}" >/dev/null 2>&1
    wait "${publisher_pid}" >/dev/null 2>&1
  fi
}
trap cleanup EXIT INT TERM

if [[ ! -e /dev/safestride-drive ]]; then
  echo "/dev/safestride-drive is missing" >&2
  exit 1
fi

if ! timeout 10s ros2 service type /walker/set_enabled \
    | grep -qx 'std_srvs/srv/SetBool'; then
  echo "/walker/set_enabled is unavailable; start scripts/run.sh first" >&2
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

ros2 topic pub --rate 20 /cmd_vel geometry_msgs/msg/TwistStamped \
  "{header: {frame_id: base_link}, twist: {linear: {x: 0.10}, angular: {z: 0.0}}}" \
  >/tmp/safestride-cmd-vel.log 2>&1 &
publisher_pid="$!"

safe_linear=""
for _ in {1..20}; do
  safe_command="$(
    timeout 2s ros2 topic echo /cmd_vel_safe --once 2>/dev/null || true
  )"
  safe_linear="$(
    awk '
      /^  linear:/ { in_linear = 1; next }
      in_linear && /^    x:/ { print $2; exit }
    ' <<<"${safe_command}"
  )"
  if [[ -n "${safe_linear}" ]] &&
      awk -v value="${safe_linear}" \
        'BEGIN { exit !(value > 0.0) }'; then
    break
  fi
  safe_linear=""
  sleep 0.1
done
if [[ -z "${safe_linear}" ]]; then
  echo "No positive /cmd_vel_safe received; check safety_supervisor output" >&2
  exit 1
fi

response="$(
  ros2 service call /walker/set_enabled std_srvs/srv/SetBool \
    "{data: true}"
)"
echo "${response}"
if ! grep -Eq 'success(=|: )[Tt]rue' <<<"${response}"; then
  echo "Drive enable request was rejected" >&2
  exit 1
fi

echo "Drive armed for ${duration}s at 0.10 m/s. Keep both wheels lifted."
echo "If encoder feedback is required, invalid feedback will fault-stop the motor."
sleep "${duration}"
echo "Test complete; sending disable command."

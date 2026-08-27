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

cleanup() {
  set +e
  timeout 3s ros2 service call /walker/set_enabled std_srvs/srv/SetBool \
    "{data: false}" >/dev/null 2>&1
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

response="$(
  ros2 service call /walker/set_enabled std_srvs/srv/SetBool \
    "{data: true}"
)"
echo "${response}"
if ! grep -Eq 'success(=|: )[Tt]rue' <<<"${response}"; then
  echo "Drive level-enable request was rejected" >&2
  exit 1
fi

echo "Dead-man direct drive allowed for ${duration}s at 0.10 m/s."
echo "Hold the pressure dead-man; releasing it disables motor output."
echo "The left D2 Hall sensor must keep producing pulses or firmware will fault-stop."
sleep "${duration}"
echo "Test complete; sending disable command."

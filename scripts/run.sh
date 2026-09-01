#!/usr/bin/env bash
set -euo pipefail

workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
config="${SAFESTRIDE_CONFIG:-${workspace}/config/raspberry_pi.yaml}"
enable_terrain="${SAFESTRIDE_ENABLE_TERRAIN:-true}"
enable_perception="${SAFESTRIDE_ENABLE_PERCEPTION:-false}"
enable_cruise="${SAFESTRIDE_ENABLE_CRUISE:-true}"
enable_crosswalk="${SAFESTRIDE_ENABLE_CROSSWALK:-true}"
enable_gps="${SAFESTRIDE_ENABLE_GPS:-true}"
gps_port="${SAFESTRIDE_GPS_PORT:-}"
if [[ -z "${gps_port}" ]]; then
  if [[ -e /dev/serial0 ]]; then
    gps_port=/dev/serial0
  elif [[ -e /dev/ttyS0 ]]; then
    gps_port=/dev/ttyS0
  else
    gps_port=/dev/serial0
  fi
fi
crosswalk_file="${SAFESTRIDE_CROSSWALK_FILE:-${workspace}/raspberry_pi/standard_crosswalks.json}"
intersection_map_file="${SAFESTRIDE_INTERSECTION_MAP_FILE:-${workspace}/raspberry_pi/v2x_intersections.json}"
default_signal_api_key_file="${workspace}/raspberry_pi/api_key.txt"
if [[ -r /etc/safestride/signal_api_key.txt ]]; then
  default_signal_api_key_file=/etc/safestride/signal_api_key.txt
fi
signal_api_key_file="${SAFESTRIDE_SIGNAL_API_KEY_FILE:-${default_signal_api_key_file}}"
intersection_id="${SAFESTRIDE_INTERSECTION_ID:-}"

check_serial_role() {
  local port="$1"
  local expected_serial="$2"
  local role="$3"
  local resolved=""
  local actual_serial=""

  if [[ ! -e "${port}" ]]; then
    echo "${role} port is missing: ${port}" >&2
    echo "Install deploy/udev/99-safestride.rules and reconnect the Uno." >&2
    exit 1
  fi
  if [[ ! -r "${port}" || ! -w "${port}" ]]; then
    echo "${role} port is not readable/writable: ${port}" >&2
    echo "Add the current user to dialout, then log in again." >&2
    exit 1
  fi
  resolved="$(readlink -f "${port}")"
  if command -v udevadm >/dev/null 2>&1; then
    actual_serial="$(
      udevadm info --query=property --name="${resolved}" 2>/dev/null |
        sed -n 's/^ID_SERIAL_SHORT=//p' || true
    )"
  fi
  if [[ -n "${actual_serial}" &&
        "${actual_serial}" != "${expected_serial}" ]]; then
    echo "${role} port points to the wrong Uno: ${port}" >&2
    echo "expected serial ${expected_serial}, got ${actual_serial}" >&2
    exit 1
  fi
  echo "${role} port: ${port} -> ${resolved} (${actual_serial:-serial unknown})"
}

check_serial_device() {
  local port="$1"
  local role="$2"
  if [[ ! -e "${port}" ]]; then
    echo "${role} port is missing: ${port}" >&2
    echo "Enable the Raspberry Pi GPIO UART or disable GPS explicitly." >&2
    exit 1
  fi
  if [[ ! -r "${port}" || ! -w "${port}" ]]; then
    echo "${role} port is not readable/writable: ${port}" >&2
    echo "Add the current user to dialout, then log in again." >&2
    exit 1
  fi
  echo "${role} port: ${port} -> $(readlink -f "${port}")"
}

if [[ "${config}" == "${workspace}/config/raspberry_pi.yaml" &&
      "${SAFESTRIDE_SKIP_PORT_CHECK:-false}" != "true" ]]; then
  check_serial_role \
    /dev/safestride-drive 75834353730351C07130 Drive
  if [[ "${enable_terrain}" == "true" ]]; then
    check_serial_role \
      /dev/safestride-terrain 8583030333935131E120 Terrain
  fi
  if [[ "${enable_gps}" == "true" ]]; then
    check_serial_device "${gps_port}" GPS
  fi
fi

if [[ "${enable_crosswalk}" == "true" && ! -r "${crosswalk_file}" ]]; then
  echo "Crosswalk map is missing or unreadable: ${crosswalk_file}" >&2
  echo "Set SAFESTRIDE_CROSSWALK_FILE or disable crosswalk monitoring." >&2
  exit 1
fi

if [[ "${enable_crosswalk}" == "true" ]]; then
  echo "Crosswalk map: ${crosswalk_file}"
  if [[ -r "${intersection_map_file}" ]]; then
    echo "Offline V2X map: ${intersection_map_file}"
  else
    echo "Offline V2X map is missing: ${intersection_map_file}" >&2
  fi
  if [[ -r "${signal_api_key_file}" ]]; then
    echo "V2X API key file: ${signal_api_key_file}"
  else
    echo "V2X API key file is missing: ${signal_api_key_file}" >&2
    echo "Crosswalk location will be visible, but signal timing is disabled." >&2
  fi
  if [[ -n "${intersection_id}" ]]; then
    echo "Static intersection fallback: ${intersection_id}" >&2
  fi
fi

set +u
source /opt/ros/jazzy/setup.bash
source "${workspace}/install/setup.bash"
set -u

if [[ "${enable_perception}" == "true" ]]; then
  # Torch/OpenCV may otherwise consume every Pi core and starve the serial and
  # safety timers that enforce command freshness.
  export OMP_NUM_THREADS="${OMP_NUM_THREADS:-1}"
  export OPENBLAS_NUM_THREADS="${OPENBLAS_NUM_THREADS:-1}"
  export MKL_NUM_THREADS="${MKL_NUM_THREADS:-1}"
  perception_venv="${SAFESTRIDE_PERCEPTION_VENV:-${workspace}/.venv-perception}"
  if [[ -x "${perception_venv}/bin/python" ]]; then
    perception_site_packages="$(
      "${perception_venv}/bin/python" -c \
        'import sysconfig; print(sysconfig.get_paths()["purelib"])'
    )"
    export PYTHONPATH="${perception_site_packages}${PYTHONPATH:+:${PYTHONPATH}}"
  fi
  if ! python3 -c 'import cv2, numpy, torch' >/dev/null 2>&1; then
    echo "Surface perception dependencies are missing." >&2
    echo "Run: bash scripts/install_perception.sh" >&2
    exit 1
  fi
fi

# ROS_LOCALHOST_ONLY=1 prevents discovery over the Ethernet cable. Jazzy's
# discovery-range setting is explicit and works for both router and direct
# cable subnets. Unset the legacy variable even if an older service env file
# still contains it.
unset ROS_LOCALHOST_ONLY
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-SUBNET}"

# The deployed config drives forward while the pressure dead-man is held.
# /walker/set_enabled false remains a manual stop until true clears the block.
launch_args=(
  "config_file:=${config}"
  "wheel_radius:=${SAFESTRIDE_WHEEL_RADIUS_M:-0.115}"
  "wheel_separation:=${SAFESTRIDE_WHEEL_SEPARATION_M:-0.55}"
  "enable_terrain:=${enable_terrain}"
  "enable_cruise:=${enable_cruise}"
  "enable_perception:=${enable_perception}"
  "perception_model_path:=${SAFESTRIDE_PERCEPTION_MODEL:-${workspace}/raspberry_pi/road_surface_inference/road_surface_public_mix_torchscript.pt}"
  "perception_classes_path:=${SAFESTRIDE_PERCEPTION_CLASSES:-${workspace}/raspberry_pi/road_surface_inference/target_classes.json}"
  "perception_camera_index:=${SAFESTRIDE_PERCEPTION_CAMERA_INDEX:-0}"
  "perception_camera_backend:=${SAFESTRIDE_PERCEPTION_CAMERA_BACKEND:-v4l2}"
  "enable_gps:=${enable_gps}"
  "gps_port:=${gps_port}"
  "enable_crosswalk:=${enable_crosswalk}"
  "crosswalk_file:=${crosswalk_file}"
  "intersection_map_file:=${intersection_map_file}"
  "signal_api_key_file:=${signal_api_key_file}"
  "enable_foxglove:=${SAFESTRIDE_ENABLE_FOXGLOVE:-false}"
)
if [[ -n "${intersection_id}" ]]; then
  launch_args+=("intersection_id:=${intersection_id}")
fi

exec ros2 launch safestride_bringup safestride.launch.py "${launch_args[@]}"

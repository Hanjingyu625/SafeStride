#!/usr/bin/env bash
set -euo pipefail

workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
config="${SAFESTRIDE_CONFIG:-${workspace}/config/raspberry_pi.yaml}"
set +u
source /opt/ros/jazzy/setup.bash
source "${workspace}/install/setup.bash"
set -u

enable_perception="${SAFESTRIDE_ENABLE_PERCEPTION:-false}"
if [[ "${enable_perception}" == "true" ]]; then
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

# Startup never calls /walker/set_enabled. Arming remains an explicit action.
exec ros2 launch safestride_bringup safestride.launch.py \
  config_file:="${config}" \
  wheel_radius:="${SAFESTRIDE_WHEEL_RADIUS_M:-0.15}" \
  wheel_separation:="${SAFESTRIDE_WHEEL_SEPARATION_M:-0.55}" \
  enable_terrain:="${SAFESTRIDE_ENABLE_TERRAIN:-true}" \
  enable_perception:="${enable_perception}" \
  perception_model_path:="${SAFESTRIDE_PERCEPTION_MODEL:-${workspace}/raspberry_pi/road_surface_inference/road_surface_public_mix_torchscript.pt}" \
  perception_classes_path:="${SAFESTRIDE_PERCEPTION_CLASSES:-${workspace}/raspberry_pi/road_surface_inference/target_classes.json}" \
  perception_camera_index:="${SAFESTRIDE_PERCEPTION_CAMERA_INDEX:-0}" \
  perception_camera_backend:="${SAFESTRIDE_PERCEPTION_CAMERA_BACKEND:-v4l2}" \
  enable_gps:="${SAFESTRIDE_ENABLE_GPS:-false}" \
  enable_crosswalk:="${SAFESTRIDE_ENABLE_CROSSWALK:-false}"

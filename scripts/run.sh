#!/usr/bin/env bash
set -euo pipefail

workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
config="${SAFESTRIDE_CONFIG:-${workspace}/config/raspberry_pi.yaml}"
source /opt/ros/jazzy/setup.bash
source "${workspace}/install/setup.bash"

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
  enable_gps:="${SAFESTRIDE_ENABLE_GPS:-false}" \
  enable_crosswalk:="${SAFESTRIDE_ENABLE_CROSSWALK:-false}"

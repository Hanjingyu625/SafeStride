#!/usr/bin/env bash
set -euo pipefail

workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
set +u
source /opt/ros/jazzy/setup.bash
set -u
cd "${workspace}"
rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy
colcon build --symlink-install --event-handlers console_cohesion+

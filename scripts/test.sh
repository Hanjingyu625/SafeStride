#!/usr/bin/env bash
set -euo pipefail

workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
cd "${workspace}"
if [[ ! -f install/setup.bash ]]; then
  echo "Run ./scripts/build.sh first." >&2
  exit 1
fi
source install/setup.bash
colcon test --event-handlers console_cohesion+
colcon test-result --verbose

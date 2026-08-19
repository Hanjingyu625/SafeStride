#!/usr/bin/env bash
set -euo pipefail

workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
set +u
source /opt/ros/jazzy/setup.bash
set -u
cd "${workspace}"
bash -n "${workspace}"/scripts/*.sh
python3 -m unittest "${workspace}/test/test_hardware_integrity.py"
if command -v arduino-cli >/dev/null 2>&1; then
  arduino-cli compile --fqbn arduino:avr:uno \
    "${workspace}/firmware/safestride_mcu"
  arduino-cli compile --fqbn arduino:avr:uno \
    "${workspace}/firmware/terrain_mcu"
else
  echo "arduino-cli not found; skipping full Uno sketch compilation." >&2
fi
if [[ ! -f install/setup.bash ]]; then
  echo "Run ./scripts/build.sh first." >&2
  exit 1
fi
set +u
source install/setup.bash
set -u
bash "${workspace}/scripts/test_firmware.sh"
colcon test --event-handlers console_cohesion+
colcon test-result --verbose

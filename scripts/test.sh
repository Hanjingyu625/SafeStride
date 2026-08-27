#!/usr/bin/env bash
set -euo pipefail

workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
cd "${workspace}"
bash -n "${workspace}"/scripts/*.sh
python3 "${workspace}/test/test_hardware_integrity.py"
python3 "${workspace}/test/test_bench_config_sync.py"
python3 "${workspace}/tools/serial_probe.py" --help >/dev/null
if [[ ! -f install/setup.bash ]]; then
  echo "Run ./scripts/build.sh first." >&2
  exit 1
fi
source install/setup.bash
bash "${workspace}/scripts/test_firmware.sh"
colcon test --event-handlers console_cohesion+
colcon test-result --verbose

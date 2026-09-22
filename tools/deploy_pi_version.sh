#!/usr/bin/env bash
set -euo pipefail

# Run on the Raspberry Pi as pi: bash /home/pi/deploy_safestride_version.sh pdj1|main
env_file=/etc/safestride/safestride.env
service=safestride.service

case "${1:-}" in
  status)
    systemctl is-active "${service}" || true
    sed -n '/^SAFESTRIDE_WORKSPACE=/p; /^SAFESTRIDE_CONFIG=/p; /^SAFESTRIDE_REQUIRE_TERRAIN_TOF=/p' "${env_file}"
    for name in pdj1 main; do
      if [[ "${name}" == pdj1 ]]; then
        checkout=/home/pi/SafeStride_jingyu
      else
        checkout=/home/pi/SafeStride-main
      fi
      git -C "${checkout}" log -1 --format="${name}: %h %s"
    done
    exit 0
    ;;
  pdj1)
    checkout=/home/pi/SafeStride_jingyu
    config=/home/pi/safestride-no-tof-pdj1.yaml
    ;;
  main)
    checkout=/home/pi/SafeStride-main
    config=/home/pi/safestride-no-tof.yaml
    ;;
  *)
    echo "Usage: $0 {pdj1|main|status}" >&2
    exit 2
    ;;
esac

version=$1
exec 9>/tmp/safestride-deploy-version.lock
flock -n 9 || { echo 'Another SafeStride deployment is running.' >&2; exit 1; }

[[ "$(id -un)" == pi ]] || { echo 'Run this script as pi.' >&2; exit 1; }
[[ -d "${checkout}/.git" && -f "${config}" ]] || {
  echo "Missing checkout or config: ${checkout}, ${config}" >&2
  exit 1
}
[[ "$(git -C "${checkout}" branch --show-current)" == "${version}" ]] || {
  echo "${checkout} is not on ${version}." >&2
  exit 1
}
git -C "${checkout}" diff --quiet
git -C "${checkout}" diff --cached --quiet
sudo -n true

# Every version keeps the deployed ToF motor controls disabled.
python3 - "${config}" <<'PY'
import sys
import yaml

settings = yaml.safe_load(open(sys.argv[1], encoding='utf-8'))
safety = settings['safety_supervisor']['ros__parameters']
for key in ('range_control_enabled', 'require_range_sensors', 'terrain_stop_enabled'):
    if safety.get(key) is not False:
        raise SystemExit(f'{sys.argv[1]}: {key} must be false')
print('ToF motor controls disabled:', sys.argv[1])
PY
grep -qx 'SAFESTRIDE_REQUIRE_TERRAIN_TOF=false' "${env_file}"

git -C "${checkout}" fetch origin "${version}"
git -C "${checkout}" merge --ff-only "origin/${version}"
echo "Building ${version} $(git -C "${checkout}" rev-parse --short HEAD)"

set +u
source /opt/ros/jazzy/setup.bash
set -u
cd "${checkout}"
MAKEFLAGS=-j2 colcon build --symlink-install --parallel-workers 2 --event-handlers console_cohesion+
[[ -f install/setup.bash && -x install/safestride_bridge/lib/safestride_bridge/serial_bridge_node ]]

candidate=$(mktemp /tmp/safestride-env.XXXXXX)
trap 'rm -f "${candidate}"' EXIT
python3 - "${env_file}" "${checkout}" "${config}" >"${candidate}" <<'PY'
from pathlib import Path
import sys

lines = Path(sys.argv[1]).read_text(encoding='utf-8').splitlines()
updates = {'SAFESTRIDE_WORKSPACE': sys.argv[2], 'SAFESTRIDE_CONFIG': sys.argv[3]}
found = set()
for index, line in enumerate(lines):
    key = line.partition('=')[0]
    if key in updates:
        lines[index] = f'{key}={updates[key]}'
        found.add(key)
for key in updates.keys() - found:
    lines.append(f'{key}={updates[key]}')
print('\n'.join(lines) + '\n', end='')
PY

backup="${env_file}.before-${version}-$(date +%Y%m%d-%H%M%S)"
sudo -n cp -p "${env_file}" "${backup}"
sudo -n install -m 0644 "${candidate}" "${env_file}.next"
sudo -n mv "${env_file}.next" "${env_file}"

if sudo -n systemctl restart "${service}"; then
  for attempt in $(seq 1 30); do
    if systemctl is-active --quiet "${service}" &&
       pgrep -f "^/usr/bin/python3 ${checkout}/install/safestride_bridge/lib/safestride_bridge/serial_bridge_node" >/dev/null &&
       pgrep -f "^/usr/bin/python3 ${checkout}/install/safestride_control/lib/safestride_control/safety_supervisor" >/dev/null; then
      echo "Active ${version}: $(git -C "${checkout}" rev-parse --short HEAD)"
      echo "Config: ${config}"
      exit 0
    fi
    sleep 1
  done
fi

echo 'New service failed its startup check; restoring the prior service configuration.' >&2
sudo -n cp -p "${backup}" "${env_file}"
sudo -n systemctl restart "${service}" || true
exit 1

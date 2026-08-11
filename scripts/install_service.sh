#!/usr/bin/env bash
set -euo pipefail

workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ ! -f "${workspace}/install/setup.bash" ]]; then
  echo "Build the workspace before installing the service." >&2
  exit 1
fi

sudo install -d -m 0755 /etc/safestride
service_user="$(id -un)"
service_group="$(id -gn)"
sed -e "s/CHANGE_ME_USER/${service_user}/" \
    -e "s/CHANGE_ME_GROUP/${service_group}/" \
    "${workspace}/deploy/systemd/safestride.service" \
    | sudo tee /etc/systemd/system/safestride.service >/dev/null

if [[ ! -f /etc/safestride/safestride.env ]]; then
  sed "s|/opt/safestride|${workspace}|g" \
    "${workspace}/deploy/systemd/safestride.env.example" \
    | sudo tee /etc/safestride/safestride.env >/dev/null
fi

sudo systemctl daemon-reload
echo "Review /etc/safestride/safestride.env and udev rules first."
echo "Then enable with: sudo systemctl enable --now safestride"

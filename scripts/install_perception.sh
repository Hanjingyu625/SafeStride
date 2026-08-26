#!/usr/bin/env bash
set -euo pipefail

workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
perception_venv="${SAFESTRIDE_PERCEPTION_VENV:-${workspace}/.venv-perception}"

python3 -m venv --system-site-packages "${perception_venv}"
"${perception_venv}/bin/python" -m pip install --upgrade pip
"${perception_venv}/bin/python" -m pip install \
  --index-url https://download.pytorch.org/whl/cpu \
  torch
"${perception_venv}/bin/python" -c 'import cv2, numpy, torch'

echo "Surface perception environment is ready: ${perception_venv}"

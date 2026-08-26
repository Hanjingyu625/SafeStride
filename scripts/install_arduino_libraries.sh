#!/usr/bin/env bash
set -euo pipefail

arduino-cli lib install "AltSoftSerial"
arduino-cli lib install "TinyGPSPlus"

echo "Terrain Uno GPS libraries are installed."

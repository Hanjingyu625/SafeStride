#!/usr/bin/env bash
set -euo pipefail

workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
build_dir="$(mktemp -d)"
trap 'rm -rf "${build_dir}"' EXIT

cxx="${CXX:-c++}"
common=(
  -std=c++11
  -Wall
  -Wextra
  -I"${workspace}/test/arduino_stub"
)

"${cxx}" "${common[@]}" \
  -I"${workspace}/firmware/safestride_mcu" \
  "${workspace}/test/firmware_protocol_test.cpp" \
  "${workspace}/firmware/safestride_mcu/protocol.cpp" \
  -o "${build_dir}/protocol_test"

"${cxx}" "${common[@]}" \
  -I"${workspace}/firmware/safestride_mcu" \
  "${workspace}/test/firmware_pressure_sensor_test.cpp" \
  "${workspace}/firmware/safestride_mcu/pressure_sensor.cpp" \
  -o "${build_dir}/pressure_test"

"${cxx}" "${common[@]}" \
  -I"${workspace}/firmware/safestride_mcu" \
  "${workspace}/test/firmware_motor_control_test.cpp" \
  "${workspace}/firmware/safestride_mcu/motor_control.cpp" \
  -o "${build_dir}/motor_test"

"${cxx}" "${common[@]}" \
  -I"${workspace}/firmware/safestride_mcu" \
  "${workspace}/test/firmware_state_machine_test.cpp" \
  "${workspace}/firmware/safestride_mcu/encoder_feedback.cpp" \
  "${workspace}/firmware/safestride_mcu/motor_control.cpp" \
  "${workspace}/firmware/safestride_mcu/pressure_sensor.cpp" \
  "${workspace}/firmware/safestride_mcu/protocol.cpp" \
  -o "${build_dir}/state_machine_test"

"${cxx}" "${common[@]}" \
  -I"${workspace}/firmware/terrain_mcu" \
  "${workspace}/test/firmware_tof10120_test.cpp" \
  "${workspace}/firmware/terrain_mcu/tof10120_sensor.cpp" \
  -o "${build_dir}/tof_test"

"${cxx}" "${common[@]}" \
  -I"${workspace}/firmware/terrain_mcu" \
  "${workspace}/test/firmware_terrain_state_test.cpp" \
  "${workspace}/firmware/terrain_mcu/gps_receiver.cpp" \
  "${workspace}/firmware/terrain_mcu/tof10120_sensor.cpp" \
  "${workspace}/firmware/terrain_mcu/protocol.cpp" \
  -o "${build_dir}/terrain_state_test"

"${build_dir}/protocol_test"
"${build_dir}/pressure_test"
"${build_dir}/motor_test"
"${build_dir}/state_machine_test"
"${build_dir}/tof_test"
"${build_dir}/terrain_state_test"

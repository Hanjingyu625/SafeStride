#!/usr/bin/env bash

# Source this file from ~/.bashrc to make the commands available everywhere:
#   source ~/SafeStride/scripts/commands.sh

_safestride_workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

cbr() {
  bash "${_safestride_workspace}/scripts/build.sh" "$@"
}

rb() {
  local workspace="${_safestride_workspace}"

  # Refuse to delete anything unless this still looks like the SafeStride root.
  if [[ ! -d "${workspace}/src" || ! -f "${workspace}/scripts/build.sh" ]]; then
    echo "rb: SafeStride workspace validation failed: ${workspace}" >&2
    return 1
  fi

  echo "Removing generated colcon directories from ${workspace}:"
  printf '  %s\n' "${workspace}/build" "${workspace}/install" "${workspace}/log"
  rm -rf -- "${workspace}/build" "${workspace}/install" "${workspace}/log"
  echo "Done."
}


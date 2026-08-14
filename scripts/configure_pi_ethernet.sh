#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  sudo bash scripts/configure_pi_ethernet.sh dhcp [interface]
  sudo bash scripts/configure_pi_ethernet.sh direct [interface]

Modes:
  dhcp    Obtain an address from a router (recommended for normal operation).
  direct  Use 10.42.0.2/24 for a direct cable to a PC at 10.42.0.1/24.

If interface is omitted, the script selects the only wired interface.
EOF
}

if [[ "${EUID}" -ne 0 ]]; then
  echo "Run this script as root (for example: sudo bash $0 direct)." >&2
  exit 1
fi

mode="${1:-}"
requested_interface="${2:-}"
if [[ "${mode}" != "dhcp" && "${mode}" != "direct" ]]; then
  usage >&2
  exit 2
fi
if [[ $# -gt 2 ]]; then
  usage >&2
  exit 2
fi

select_wired_interface() {
  local path name
  local -a candidates=()

  for path in /sys/class/net/*; do
    name="${path##*/}"
    [[ "${name}" == "lo" ]] && continue
    [[ -d "${path}/wireless" ]] && continue
    [[ -e "${path}/device" ]] || continue
    candidates+=("${name}")
  done

  if [[ ${#candidates[@]} -eq 1 ]]; then
    printf '%s\n' "${candidates[0]}"
    return
  fi

  if [[ ${#candidates[@]} -eq 0 ]]; then
    echo "No wired network interface was detected." >&2
  else
    echo "Multiple wired interfaces were detected: ${candidates[*]}" >&2
    echo "Pass the intended interface name as the second argument." >&2
  fi
  return 1
}

interface="${requested_interface:-$(select_wired_interface)}"
if [[ ! "${interface}" =~ ^[A-Za-z0-9_.-]+$ || "${interface}" == "lo" ]]; then
  echo "Unsafe or invalid interface name: ${interface}" >&2
  exit 2
fi
if [[ ! -d "/sys/class/net/${interface}" || -d "/sys/class/net/${interface}/wireless" ]]; then
  echo "Wired interface does not exist: ${interface}" >&2
  exit 2
fi

for command in ip install mktemp netplan systemctl; do
  if ! command -v "${command}" >/dev/null 2>&1; then
    echo "Required command is missing: ${command}" >&2
    exit 1
  fi
done

target="/etc/netplan/90-safestride-ethernet.yaml"
timestamp="$(date -u +%Y%m%dT%H%M%SZ)"
backup=""
temporary="$(mktemp)"
trap 'rm -f "${temporary}"' EXIT

if [[ -f "${target}" ]]; then
  backup="${target}.${timestamp}.bak"
  cp --preserve=mode,ownership,timestamps "${target}" "${backup}"
fi

if [[ "${mode}" == "dhcp" ]]; then
  cat >"${temporary}" <<EOF
network:
  version: 2
  ethernets:
    ${interface}:
      dhcp4: true
      optional: true
EOF
else
  cat >"${temporary}" <<EOF
network:
  version: 2
  ethernets:
    ${interface}:
      dhcp4: false
      addresses:
        - 10.42.0.2/24
      optional: true
EOF
fi

restore_previous_config() {
  echo "Restoring the previous SafeStride Netplan configuration." >&2
  if [[ -n "${backup}" ]]; then
    cp "${backup}" "${target}"
  else
    rm -f "${target}"
  fi
  netplan generate >/dev/null 2>&1 || true
  netplan apply >/dev/null 2>&1 || true
}

install -o root -g root -m 600 "${temporary}" "${target}"
if ! netplan generate; then
  restore_previous_config
  exit 1
fi

echo "Applying Netplan to ${interface}. An existing Ethernet SSH session may disconnect."
if ! netplan apply; then
  restore_previous_config
  exit 1
fi

systemctl enable --now ssh.service avahi-daemon.service
ip link set dev "${interface}" up

hostname_value="$(hostname)"
echo
echo "SafeStride Ethernet setup complete."
echo "  mode:      ${mode}"
echo "  interface: ${interface}"
ip -brief -4 address show dev "${interface}"
if [[ "${mode}" == "direct" ]]; then
  echo "Connect from the PC with: ssh ${SUDO_USER:-ubuntu}@10.42.0.2"
else
  echo "Connect with: ssh ${SUDO_USER:-ubuntu}@${hostname_value}.local"
  echo "If .local is unavailable, use the IPv4 address printed above."
fi

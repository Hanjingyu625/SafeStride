#!/usr/bin/env bash
set -u

echo "== uptime / load =="
uptime
echo "== Pi power and thermal flags =="
if command -v vcgencmd >/dev/null 2>&1; then
  vcgencmd get_throttled
  vcgencmd measure_temp
else
  echo "vcgencmd unavailable"
fi
echo "== network links =="
ip -brief address
ip -s link
echo "== Wi-Fi link / power save =="
if command -v iw >/dev/null 2>&1; then
  iw dev
  for interface in /sys/class/net/wl*; do
    [[ -e "${interface}" ]] || continue
    iw dev "$(basename "${interface}")" link
    iw dev "$(basename "${interface}")" get power_save
  done
fi
echo "== recent undervoltage, USB, Wi-Fi and SSH events =="
journalctl -b --no-pager -n 300 2>/dev/null |
  grep -Ei 'under.?voltage|voltage|usb|ttyACM|wlan|wifi|disconnect|sshd' || true
echo "== SafeStride service =="
systemctl --no-pager --full status safestride.service 2>/dev/null || true

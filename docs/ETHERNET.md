# Raspberry Pi Ethernet and remote ROS 2

SafeStride supports either a normal router/switch connection using DHCP or a
direct Ethernet cable between a Windows PC and the Raspberry Pi. The ROS 2
domain is `42`, and discovery is enabled on the local subnet. Motors remain
disarmed after boot and must still be enabled explicitly.

## One-time Raspberry Pi preparation

For the first setup, use a monitor/keyboard or an already working Wi-Fi/SSH
connection. On Ubuntu Server 24.04:

```bash
cd ~/SafeStride
bash scripts/install_ubuntu_24_04.sh
```

The installer installs and enables OpenSSH and Avahi, so the Pi can normally be
reached as `<hostname>.local`. Log out and back in once after installation.

## Option A: connect through a router or switch

Connect both computers to the same LAN, then run on the Pi:

```bash
cd ~/SafeStride
sudo bash scripts/configure_pi_ethernet.sh dhcp
```

If the Pi has more than one wired interface, append its name, for example
`sudo bash scripts/configure_pi_ethernet.sh dhcp eth0`. Connect from Windows:

```powershell
ssh ubuntu@raspberrypi.local
```

If mDNS is unavailable, use the IPv4 address printed by the configuration
script or by `ip -brief -4 address`.

## Option B: direct PC-to-Pi cable

Modern Ethernet ports normally auto-negotiate, so a regular cable is enough.
First run this on the Pi from its local console (or while connected over
Wi-Fi):

```bash
cd ~/SafeStride
sudo bash scripts/configure_pi_ethernet.sh direct
```

This assigns `10.42.0.2/24` to the Pi without a default gateway, so an existing
Wi-Fi Internet route remains unchanged. Then open Windows PowerShell as
Administrator, find the adapter name with `Get-NetAdapter`, and run:

```powershell
cd C:\path\to\SafeStride
.\scripts\windows\configure_direct_ethernet.ps1 -InterfaceAlias 'Ethernet' -Mode Direct
ping 10.42.0.2
ssh ubuntu@10.42.0.2
```

The Windows script assigns `10.42.0.1/24` only to the selected adapter and adds
an inbound UDP firewall rule limited to the direct-link addresses for ROS 2 DDS.
It does not add a gateway. To return that adapter to DHCP:

```powershell
.\scripts\windows\configure_direct_ethernet.ps1 -InterfaceAlias 'Ethernet' -Mode Dhcp
```

## Build and run remotely

After cloning this repository to `/home/ubuntu/SafeStride`, the Windows helper
can run common SSH operations. Replace `ubuntu` with the actual Pi account:

```powershell
.\scripts\remote_pi.ps1 -PiUser ubuntu -Action Info
.\scripts\remote_pi.ps1 -PiUser ubuntu -Action Build
.\scripts\remote_pi.ps1 -PiUser ubuntu -Action Test
.\scripts\remote_pi.ps1 -PiUser ubuntu -Action Run
```

`Run` stays attached to the SSH terminal; press Ctrl+C to stop it. For boot-time
operation, install and manage the service:

```powershell
.\scripts\remote_pi.ps1 -PiUser ubuntu -Action InstallService
.\scripts\remote_pi.ps1 -PiUser ubuntu -Action Status
.\scripts\remote_pi.ps1 -PiUser ubuntu -Action Logs
.\scripts\remote_pi.ps1 -PiUser ubuntu -Action Restart
```

For a router-assigned address, add `-PiHost <hostname>.local` or the Pi address.
Use `-Workspace /another/absolute/path` if the repository is elsewhere.

## Inspect ROS 2 over the cable

The simplest and most predictable method is to run the ROS CLI on the Pi over
SSH:

```powershell
.\scripts\remote_pi.ps1 -PiUser ubuntu -Action Topics
ssh ubuntu@10.42.0.2 "bash -lc 'source /opt/ros/jazzy/setup.bash; source ~/SafeStride/install/setup.bash; ros2 topic echo /handle/pressure --once'"
```

If ROS 2 Jazzy is also installed on the PC, use the same discovery settings in
that terminal:

```powershell
Remove-Item Env:ROS_LOCALHOST_ONLY -ErrorAction SilentlyContinue
$env:ROS_DOMAIN_ID = '42'
$env:ROS_AUTOMATIC_DISCOVERY_RANGE = 'SUBNET'
ros2 node list
ros2 topic list
```

The service environment and `scripts/run.sh` apply those same settings on the
Pi. `scripts/run.sh` also removes the legacy `ROS_LOCALHOST_ONLY` variable, so
an older `/etc/safestride/safestride.env` cannot accidentally restrict ROS to
loopback.

## Troubleshooting

- Confirm link and addresses with `ip -brief -4 address` on the Pi and
  `Get-NetIPAddress -InterfaceAlias 'Ethernet' -AddressFamily IPv4` on Windows.
- Confirm SSH first with `ssh -v ubuntu@10.42.0.2`. ROS troubleshooting is
  premature until SSH works.
- Both ROS terminals must use domain `42`, and neither may have
  `ROS_LOCALHOST_ONLY=1`.
- Restart a stale ROS CLI daemon with `ros2 daemon stop`, then retry the list or
  echo command.
- If multicast discovery is blocked, set `ROS_STATIC_PEERS=10.42.0.1` on the Pi
  and `ROS_STATIC_PEERS=10.42.0.2` on the PC before starting nodes.
- Check the service with `systemctl status safestride.service` and
  `journalctl -u safestride.service -n 200 --no-pager`.
- The Pi firewall, if enabled separately, must allow DDS UDP traffic on the
  trusted Ethernet subnet. Do not expose DDS directly to an untrusted network.

References: [Ubuntu OpenSSH server documentation](https://documentation.ubuntu.com/server/how-to/security/openssh-server/),
[ROS 2 improved dynamic discovery](https://docs.ros.org/en/rolling/Tutorials/Advanced/Improved-Dynamic-Discovery.html),
and [Netplan documentation](https://netplan.readthedocs.io/).

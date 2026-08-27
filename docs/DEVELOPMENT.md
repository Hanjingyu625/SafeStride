# Development workflow

For wired Raspberry Pi access, direct-cable addressing, SSH helpers, and ROS 2
subnet discovery, follow [ETHERNET.md](ETHERNET.md) before the installation
steps below.

The production target is Raspberry Pi 4 running 64-bit Ubuntu Server 24.04
(Noble), ROS 2 Jazzy and Python 3.12. Windows is an editing and host-unit-test
environment; release acceptance happens on arm64 Linux.

## Raspberry Pi installation

```bash
git clone <repository-url> ~/SafeStride
cd ~/SafeStride
bash scripts/install_ubuntu_24_04.sh
# Log out and back in after group membership changes.
bash scripts/install_arduino_libraries.sh
bash scripts/build.sh
bash scripts/test.sh
```

The installer intentionally does not flash firmware, install udev rules, enable
systemd or arm a motor.

```bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
colcon test
colcon test-result --verbose
```

Flash one Uno at a time with motor-driver 12 V power isolated. Use stable paths
under `/dev/serial/by-id/`; never assume `ttyACM0` ordering.

Copy and edit `deploy/udev/99-safestride.rules.example`, then install it only
after checking the unique serial attribute of each device. The runtime config
expects `/dev/safestride-drive` and `/dev/safestride-terrain`. The default GPS
path is BE-220 -> Terrain Uno D8 -> `/dev/safestride-terrain`; the separate
Pi GPS serial node is only a fallback and must not run at the same time.

Both Arduino sketches must be flashed after a wire-protocol change. Protocol
v4 is intentionally incompatible with older firmware, so
the Drive MCU, Terrain MCU and ROS bridge must be updated together.

For unattended startup, first review `config/raspberry_pi.yaml`, install the
udev rules, build successfully, and then run `bash scripts/install_service.sh`.
Drive enable is level-triggered by live safety inputs after startup.
Keep cruise disabled during initial sensor and lifted-wheel tests.

Put the supplied shapefile in `data/external/crosswalk_shp/` and converted JSON
in `data/generated/`. Store YOLO weights in GitHub Releases or an artifact store
and record their SHA-256 checksum.

The ZIP's latest `smart_crosswalk_controller_v6.py` has been migrated into
`safestride_sensors` and `safestride_navigation`; do not run the standalone
controller alongside ROS. See `docs/CROSSWALK.md` for data conversion,
`itstId`, API-key and monitor-only setup.

## Road-surface perception

Ubuntu Server has no desktop requirement. Runtime perception must not call GUI
functions such as `cv2.imshow`. The current prototype uses a CPU TorchScript
model and a USB camera through Linux V4L2. Install its isolated Python
environment. Perception is disabled by default. Enable it explicitly only after
the sensor-only and lifted-wheel tests:

```bash
bash scripts/install_perception.sh
SAFESTRIDE_ENABLE_PERCEPTION=true bash scripts/run.sh
```

Set `SAFESTRIDE_PERCEPTION_CAMERA_INDEX` if the camera is not `/dev/video0`.
The perception node publishes `/perception/surface_condition`; when enabled,
the safety supervisor requires a fresh valid message and applies its speed
scale before `/cmd_vel_safe` reaches the Drive serial bridge. Smooth pavement
may request up to 1.20x, but the final command remains clamped by the absolute
0.15 m/s limit. Camera failures,
low confidence and stale inference therefore inhibit motion. Benchmark
worst-case latency and classification errors on the Pi before loaded tests.

The GitHub workflow builds Jazzy on Ubuntu 24.04 amd64. It catches ROS API and
packaging errors; final arm64 performance and device tests still run on the Pi.

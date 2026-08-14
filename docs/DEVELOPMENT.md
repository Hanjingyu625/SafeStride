# Development workflow

The production target is Raspberry Pi 4 running 64-bit Ubuntu Server 24.04
(Noble), ROS 2 Jazzy and Python 3.12. Windows is an editing and host-unit-test
environment; release acceptance happens on arm64 Linux.

## Raspberry Pi installation

```bash
git clone <repository-url> ~/SafeStride
cd ~/SafeStride
bash scripts/install_ubuntu_24_04.sh
# Log out and back in after group membership changes.
bash scripts/build.sh
bash scripts/test.sh
```

For short build and clean commands, load the project commands once in your
shell startup file:

```bash
echo 'source ~/SafeStride/scripts/commands.sh' >> ~/.bashrc
source ~/.bashrc
```

After that, `cbr` builds the workspace using `scripts/build.sh`, and `rb`
removes only the generated colcon directories (`build/`, `install/`, and
`log/`). Both commands work from any directory. Extra arguments passed to
`cbr` are forwarded to `colcon build`.

The installer intentionally does not flash firmware, install udev rules, enable
systemd or arm a motor.

```bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
colcon test
colcon test-result --verbose
```

Flash one Uno at a time with wheel/leg motor power isolated. Use stable paths
under `/dev/serial/by-id/`; never assume `ttyACM0` ordering.

Copy and edit `deploy/udev/99-safestride.rules.example`, then install it only
after checking the unique serial attribute of each device. The runtime config
expects `/dev/safestride-drive`; terrain and GPS nodes will use their respective
aliases when implemented.

For unattended startup, first review `config/raspberry_pi.yaml`, install the
udev rules, build successfully, and then run `bash scripts/install_service.sh`.
The service starts disarmed and never calls the enable service automatically.

Put the supplied shapefile in `data/external/crosswalk_shp/` and converted JSON
in `data/generated/`. Store YOLO weights in GitHub Releases or an artifact store
and record their SHA-256 checksum.

The ZIP's latest `smart_crosswalk_controller_v6.py` has been migrated into
`safestride_sensors` and `safestride_navigation`; do not run the standalone
controller alongside ROS. See `docs/CROSSWALK.md` for data conversion,
`itstId`, API-key and monitor-only setup.

## Camera and YOLO

Ubuntu Server has no desktop requirement. Runtime perception must not call GUI
functions such as `cv2.imshow`. The model backend is not installed yet because
the camera and model export format have not been selected. Prefer a small ONNX
model, pin its checksum, measure worst-case latency on the Pi, and publish an
invalid/zero speed scale whenever inference becomes stale.

The GitHub workflow builds Jazzy on Ubuntu 24.04 amd64. It catches ROS API and
packaging errors; final arm64 performance and device tests still run on the Pi.

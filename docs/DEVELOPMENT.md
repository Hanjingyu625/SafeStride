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
expects `/dev/safestride-drive`, `/dev/safestride-terrain` and a GPIO UART.
`scripts/run.sh` selects `/dev/serial0` first and falls back to `/dev/ttyS0`;
`SAFESTRIDE_GPS_PORT` overrides both. The GPS path is BE-220 -> Raspberry Pi
GPIO serial -> `gps_node`; Terrain Uno does not receive or relay GPS data.

Both Arduino sketches must be flashed after a wire-protocol change. Protocol
v4 is intentionally incompatible with older firmware, so
the Drive MCU, Terrain MCU and ROS bridge must be updated together.

For unattended startup, first review `config/raspberry_pi.yaml`, install the
udev rules, build successfully, and then run `bash scripts/install_service.sh`.
The deployed config uses dead-man direct drive. While both pressure channels
are active and the Drive link has no MCU fault, the bridge streams a fixed
0.10 m/s forward target without waiting for `/cmd_vel_safe`. Releasing either
pressure input immediately sends a disabled stop. Set
`command.deadman_direct_drive` to `false` to restore supervised velocity input.
Drive firmware also keeps Hall feedback telemetry-only while
`DEADMAN_DIRECT_DRIVE=true`; it does not delay re-arming or latch a Hall fault.
Keep the wheels lifted during initial tests.

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

### Retraining the surface model

Use `notebooks/road_surface_training_colab.ipynb` with a Colab GPU. The
notebook calls `tools/train_road_surface.py`, caches public datasets in Google
Drive, and fine-tunes MobileNetV3-Small as the default Raspberry Pi model.
Its inspection cells display the exact dataset, DataLoader, training and
quantization functions executed by the launcher cell.
One unavailable public source is reported without discarding the usable data.
Training requires a hard floor of 60 valid images per class and warns below the
recommended 250; grouped validation and test splits still require 10 examples
per class. Splits are grouped by capture sequence or location to reduce frame
leakage. The learning rate is reduced when validation macro F1 plateaus and
early stopping selects the actual epoch count.

The prepared dataset manifest and every completed epoch are persisted under
`MyDrive/SafeStride`. Re-running the notebook with the same configuration
reuses the manifest and resumes the compatible checkpoint. A browser closure
therefore loses at most the currently running epoch; deleting the checkpoint
directory intentionally starts training from the ImageNet initialization.

FX post-training static INT8 uses QNNPACK qint8 weights and quint8 activations.
Observer calibration uses up to 32 batches from a deterministic, class-balanced
train-only loader with evaluation transforms; validation and test samples are
excluded. INT8 is accepted only when validation macro F1 drops by at most 0.015
and no class recall drops by more than 0.05. Pruning is intentionally not used
because sparse weights alone do not guarantee faster dense ARM inference.
A production model is approved only at test macro F1 >= 0.75 and per-class
recall >= 0.55. Failed candidates and their metrics are preserved under a
candidate filename, but must not replace the ROS model.

The final artifacts are:

- `road_surface_public_mix_torchscript.pt`
- `target_classes.json`
- `model_manifest.json`
- `training_report.json`
- `dataset_manifest.csv`

At runtime the classifier publishes exactly one top-1 class. Predictions below
0.65 confidence or with less than a 0.15 gap over the runner-up are invalid and
request a zero speed scale. Torch and OpenCV are limited to one CPU thread in
the Pi configuration.

For extra SafeStride camera data, arrange images as
`CLASS_NAME/CAPTURE_GROUP/image.jpg` and pass `--local-data-dir`. Keep one
continuous video or walking route in one capture group. Never split adjacent
video frames between training and validation.

Copy a model into `raspberry_pi/road_surface_inference/` only after reviewing
the manifest's test macro F1, every class recall, artifact hash, model size and
CPU latency on the Pi. The exported input contract remains RGB 224x224 with
ImageNet normalization, matching the ROS perception node.

The surface scale multiplies the ROS velocity command; it is not raw Arduino
PWM. With the current 0.15 m wheel radius, the 0.08 m/s default request is about
533 mrad/s. Open-loop firmware maps that non-zero target to PWM 92, above the
tested PWM 90 motor dead zone. Because the temporary open-loop range is only
PWM 90 through 100, the current 0.4x through 1.2x surface scales produce only a
small electrical-output difference around the default cruise speed. Reliable
surface-dependent physical speed requires encoder feedback or a separately
validated wider PWM range; do not lower the start threshold merely to make the
numbers look farther apart.

The GitHub workflow builds Jazzy on Ubuntu 24.04 amd64. It catches ROS API and
packaging errors; final arm64 performance and device tests still run on the Pi.

# SafeStride

SafeStride is a ROS 2 smart-walker workspace targeting a Raspberry Pi 4 with
64-bit Ubuntu Server 24.04 (Noble), ROS 2 Jazzy and Python 3.12.

> This repository is a development scaffold, not a certified safety controller.
> All motor outputs remain disabled until wiring, limits, polarity and watchdogs
> are validated with the mechanism unloaded. Never begin testing with a person
> supported by the walker.

## System

```text
BE-220 GPS -----> Raspberry Pi 4 <----- camera / YOLO
                         |
              ROS 2 safety supervisor
                    /           \
         USB serial               USB serial
        Drive Uno                Terrain Uno
   wheels / encoders /       TOF / MPU / BNO055 /
   pressure / E-stop         step-leg actuator
```

- Drive Uno has final authority over the two wheel motors.
- Terrain Uno has final authority over the step-leg actuator.
- YOLO is advisory and can only reduce permitted speed.
- A timeout, invalid sensor, unknown surface or lost serial session produces a
  stop or zero speed scale.
- Startup never arms an actuator automatically.

See [architecture](docs/ARCHITECTURE.md), [hardware](docs/HARDWARE.md),
[development](docs/DEVELOPMENT.md), and [roadmap](docs/ROADMAP.md).

## Raspberry Pi 4 quick start

```bash
git clone <repository-url> ~/SafeStride
cd ~/SafeStride
bash scripts/install_ubuntu_24_04.sh
```

Log out and back in after the installer changes `dialout` and `video` groups.

```bash
cd ~/SafeStride
bash scripts/build.sh
bash scripts/test.sh
```

Before running, replace the serial placeholders with verified udev identities
and install the rules described in [development](docs/DEVELOPMENT.md). Then:

```bash
bash scripts/run.sh
```

The default production configuration requires valid range sensing and uses
`/dev/safestride-drive`. It should therefore refuse motion on an incomplete
bench setup.

## Repository layout

```text
src/                         ROS 2 Jazzy packages
firmware/safestride_mcu/     Drive Uno firmware
firmware/terrain_mcu/        Terrain Uno safe scaffold
config/                      Hardware and runtime configuration
deploy/udev/                 Stable serial-device aliases
deploy/systemd/              Headless startup service
scripts/                     Noble/Jazzy install, build, test and run
docker/                      Jazzy development image
data/                        External and generated GIS data (not committed)
models/                      Model metadata; weights are external artifacts
test/                        Host-side firmware tests
```

ROS packages are divided by responsibility rather than one package per sensor:

- `safestride_interfaces`
- `safestride_bridge`
- `safestride_control`
- `safestride_sensors`
- `safestride_perception`
- `safestride_terrain`
- `safestride_bringup`
- `safestride_description`

## Current limitations

- TOF-10120, MPU-9250 and BNO055 hardware drivers are not implemented yet.
- The step-leg pin map, driver and limit switches are not selected.
- BE-220 parsing exists as tested library code, but its ROS node is pending.
- YOLO runtime awaits camera selection, dataset and an exported model.
- Crosswalk v6 ZIP code remains migration reference and is not started by ROS.
- Wheel dimensions, encoder resolution, PID and pressure thresholds are example
  values and must be measured.

## Team workflow

Use feature branches and pull requests. GitHub Actions builds and tests ROS 2
Jazzy on Ubuntu 24.04. The Raspberry Pi remains the required ARM64, camera,
serial and real-time performance acceptance platform.

Do not commit API keys, personal GPS tracks, raw camera recordings, YOLO weights
or the large source shapefiles. See [CONTRIBUTING.md](CONTRIBUTING.md).

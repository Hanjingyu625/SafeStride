#!/usr/bin/env bash
set -euo pipefail

if [[ "$(. /etc/os-release && echo "${ID}:${VERSION_ID}")" != "ubuntu:24.04" ]]; then
  echo "This installer supports Ubuntu 24.04 only." >&2
  exit 1
fi

case "$(uname -m)" in
  aarch64|arm64) ;;
  *) echo "Warning: production target is Raspberry Pi 4 arm64; found $(uname -m)." >&2 ;;
esac

sudo apt-get update
sudo apt-get install -y curl software-properties-common
sudo add-apt-repository -y universe

ros_apt_version="$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | sed -n 's/.*"tag_name": "\([^"]*\)".*/\1/p')"
if [[ -z "${ros_apt_version}" ]]; then
  echo "Could not determine ros2-apt-source version." >&2
  exit 1
fi
codename="$(. /etc/os-release && echo "${VERSION_CODENAME}")"
deb="/tmp/ros2-apt-source.deb"
curl -fsSL -o "${deb}" "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ros_apt_version}/ros2-apt-source_${ros_apt_version}.${codename}_all.deb"
sudo dpkg -i "${deb}"

sudo apt-get update
sudo apt-get install -y \
  avahi-daemon \
  build-essential \
  netplan.io \
  openssh-server \
  ros-jazzy-ros-base \
  ros-dev-tools \
  python3-colcon-common-extensions \
  python3-numpy \
  python3-opencv \
  python3-rosdep \
  python3-serial \
  python3-venv \
  v4l-utils

if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  sudo rosdep init
fi
rosdep update
sudo usermod -aG dialout,video "${USER}"
sudo systemctl enable --now ssh.service avahi-daemon.service

echo "Installation complete. Log out and back in for dialout/video groups."
echo "Then run: ./scripts/build.sh"
echo "For road-surface inference, also run: ./scripts/install_perception.sh"

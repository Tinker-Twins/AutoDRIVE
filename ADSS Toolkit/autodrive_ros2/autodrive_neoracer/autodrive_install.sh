#!/bin/bash

################################################################################

# Copyright (c) 2026, Tinker Twins, AutoDRIVE Ecosystem
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

################################################################################

# AutoDRIVE Devkit Setup Script for NeoRacer (ROS 2 Humble)
#
# This script sets up the AutoDRIVE-NeoRacer Devkit environment on Ubuntu 22.04
# with ROS 2 Humble substrate. It installs necessary dependencies, configures user
# environment, and sets up services for AutoDRIVE-NeoRacer and related components.
#
# Usage: sudo bash autodrive_install.sh

################################################################################

set -e

# Check root access
if [ "$EUID" -ne 0 ]; then echo "Please run as root: sudo bash autodrive_install.sh"; exit 1; fi

# Set environment variables
TARGET_USER="${SUDO_USER:-$(id -un)}"
HOME_DIR=$(eval echo "~$TARGET_USER")
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Prevent apt stops at interactive prompt
export DEBIAN_FRONTEND=noninteractive

# Check if PEP 668 flag exists for pip
PIP_FLAGS=()
pip3 install --help 2>/dev/null | grep -q break-system-packages && PIP_FLAGS+=(--break-system-packages)

################################################################################

echo ">> [1/6] Installing ROS 2 Humble Hawksbill (LTS)"

# Set locale
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Setup sources
sudo apt install -y software-properties-common
sudo add-apt-repository universe -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

# Install ROS 2 packages
sudo apt update
sudo apt upgrade -y
sudo apt install -y ros-humble-desktop # Desktop install (recommended)
# sudo apt install -y ros-humble-ros-base # Base Install (bare bones)
sudo apt install -y ros-dev-tools

# Install additional ROS 2 packages
sudo apt update && apt install -y --no-install-recommends \
  ros-humble-tf-transformations \
  ros-humble-imu-tools \
  ros-humble-ackermann-msgs \
  ros-humble-cv-bridge \
  ros-humble-robot-localization \
  ros-humble-imu-complementary-filter \
  ros-humble-slam-toolbox \
  ros-humble-libg2o \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-joint-state-publisher \
  ros-humble-diagnostic-updater \
  ros-humble-vision-msgs \
  ros-humble-image-transport-plugins \
  ros-humble-rosbag2-storage-mcap

################################################################################

echo ">> [2/6] Installing Python Dependencies"

# AutoDRIVE Devkit's Python dependencies
pip3 install attrdict
pip3 install numpy==1.22.2
pip3 install pillow
pip3 install opencv-contrib-python==4.10.0.84
pip3 install eventlet==0.33.3
pip3 install Flask==1.1.1
pip3 install Flask-SocketIO==4.1.0
pip3 install python-socketio==4.2.0
pip3 install python-engineio==3.13.0
pip3 install greenlet==1.1.0
pip3 install gevent==21.12.0
pip3 install gevent-websocket==0.10.1
pip3 install Jinja2==3.0.3
pip3 install itsdangerous==2.0.1
pip3 install werkzeug==2.0.3
pip3 install transforms3d

# NeoRacer Python dependencies
pip3 install "${PIP_FLAGS[@]}" luma.led_matrix

################################################################################

echo ">> [3/6] Setting Up User Environment"

# Emulate Jetson thermal management system

JETSON_THERMAL="/jetson_thermal"
mkdir -p "$JETSON_THERMAL"
zones=(
    "cpu-thermal:43531"
    "gpu-thermal:45750"
    "cv0-thermal:46467"
    "cv1-thermal:48235"
    "cv2-thermal:47732"
    "soc0-thermal:49562"
    "soc1-thermal:49031"
    "soc2-thermal:48343"
    "tj-thermal:50937"
)
for i in "${!zones[@]}"; do
    IFS=':' read -r type temp <<< "${zones[$i]}"
    zone="$JETSON_THERMAL/thermal_zone$i"
    mkdir -p "$zone"
    echo "$type" > "$zone/type"
    echo "$temp" > "$zone/temp"
done
echo "Jetson thermal filesystem created:"
find "$JETSON_THERMAL" -type f -print



sudo -u "$TARGET_USER" bash <<USEREOF
set -e
cd "$HOME_DIR"

# FastDDS UDP-only profile: prevents one-way silent failures caused by stale or
# cross-user shared-memory segments (especially on WSL2)

cp "$SCRIPT_DIR/fastdds_udp.xml" "$HOME_DIR/fastdds_udp.xml"

# Student library + labs, laid out exactly as setup_jupyter.sh does on the car:
# ~/jupyter_ws/neoracer-os/{library,labs} with the library selected through a
# racecar_student.pth in the user site-packages.

NEO_OS="$HOME_DIR/jupyter_ws/neoracer-os"
TMP=\$(mktemp -d)
git clone --depth 1 --branch main https://github.com/Neobotics-Foundation-Inc/racecar-neo-library "\$TMP/lib"
git clone --depth 1 https://github.com/Neobotics-Foundation-Inc/neoracer-labs "\$TMP/labs"
mkdir -p "\$NEO_OS"
rm -rf "\$NEO_OS/library"
cp -r "\$TMP/lib/library" "\$NEO_OS/library"
rm -rf "\$TMP/labs/.git"
mkdir -p "\$NEO_OS/labs"
cp -r "\$TMP/labs/." "\$NEO_OS/labs/"
rm -rf "\$TMP"
cp "$SCRIPT_DIR/scripts/disparity_extender.py" "\$NEO_OS/labs/"

# AutoDRIVE-specific: the wall follower's RB deadman guards fragile hardware;
# AutoDRIVE has none, so disable it in THIS copy. The repository default stays True.

[ -f "\$NEO_OS/labs/ultimate-wall-follower/wall_follower.py" ] && \
  sed -i "s/^REQUIRE_DEADMAN = True/REQUIRE_DEADMAN = False/" \
    "\$NEO_OS/labs/ultimate-wall-follower/wall_follower.py"
SITE=\$(python3 -c "import site; print(site.getusersitepackages())")
mkdir -p "\$SITE"
echo "\$NEO_OS/library" > "\$SITE/racecar_student.pth"

# ROS 2 workspace: the AutoDRIVE bridge plus the REAL driver package, whose mux
# and throttle nodes run unmodified (autodrive.launch.py starts them).

mkdir -p ros2_ws/src

# This script ships in two layouts: AutoDRIVE's Devkit/ (package in
# an autodrive_neoracer/ subdirectory) and the AutoDRIVE-Devkit toolkit
# (package.xml beside this script). Link whichever holds the package.

if [ -f "$SCRIPT_DIR/package.xml" ]; then
    ln -sfn "$SCRIPT_DIR" ros2_ws/src/autodrive_neoracer
else
    ln -sfn "$SCRIPT_DIR/autodrive_neoracer" ros2_ws/src/autodrive_neoracer
fi
[ -d "$HOME_DIR/neoracer_ros2_driver" ] || \
  git clone --depth 1 https://github.com/Neobotics-Foundation-Inc/neoracer_ros2_driver.git "$HOME_DIR/neoracer_ros2_driver"

# Repo root, not the inner package dir: the car keeps the whole repo at
# src/neoracer_ros2_driver so scripts/racecar-tool.sh resolves from the same
# path. colcon discovers the nested package either way.

ln -sfn "$HOME_DIR/neoracer_ros2_driver" ros2_ws/src/neoracer_ros2_driver
source /opt/ros/humble/setup.bash
cd ros2_ws && colcon build --symlink-install --packages-select autodrive_neoracer neoracer_ros2_driver
cd "$HOME_DIR"

# osracer vendor stack. Pinned to a commit on the vendor's product/neo line:
# osrbot has deleted branches under us before (dev, 2026-08), and AutoDRIVE
# must build the tree it was validated against. Bump deliberately.

mkdir -p osracer_ws/src
[ -d osracer_ws/src/osracer ] || \
  git clone -q --recurse-submodules --single-branch --branch product/neo https://github.com/osrbot/osracer.git osracer_ws/src/osracer
cd osracer_ws && colcon build --symlink-install
cd "$HOME_DIR"

# launchers (fallbacks; the canonical workflow is the bashrc aliases below)

cp "$SCRIPT_DIR/start_autodrive_bridge.sh" "$HOME_DIR/start_autodrive_bridge.sh"
cp "$SCRIPT_DIR/run_autodrive_lab.sh" "$HOME_DIR/run_autodrive_lab.sh"
chmod +x "$HOME_DIR/start_autodrive_bridge.sh" "$HOME_DIR/run_autodrive_lab.sh"

# Jetson-parity shell environment: same structure as the real NeoRacer, where
# .bashrc sources ROS + workspace + library and 'teleop' is a launch alias.

if ! grep -q "NeoRacer AutoDRIVE" "$HOME_DIR/.bashrc"; then
cat >> "$HOME_DIR/.bashrc" <<'BLOCK'

# >>> NeoRacer AutoDRIVE (mirrors the Jetson bashrc) >>>

source /opt/ros/humble/setup.bash

# osracer underlay BENEATH the neoracer overlay, same order as the car's
# launch_autonomy.sh
# NOTE: this BLOCK sits inside the outer USEREOF heredoc, which expands
# variables as root - every literal-$HOME below must stay escaped.

[ -f \$HOME/osracer_ws/install/setup.bash ] && source \$HOME/osracer_ws/install/setup.bash
source \$HOME/ros2_ws/install/setup.bash
export FASTRTPS_DEFAULT_PROFILES_FILE=\$HOME/fastdds_udp.xml

# For every shell to default to UDP-only DDS (optional)
# echo 'export FASTRTPS_DEFAULT_PROFILES_FILE=\$HOME/fastdds_udp.xml' >> ~/.bashrc
# source ~/.bashrc

# 'teleop' on the car brings up the driver; 'autodrive' brings up the
# AutoDRIVE bridge + mux + throttle.

alias autodrive='ros2 launch autodrive_neoracer autodrive.launch.py'

# the car's autonomy graph with ground-truth TF (slam:=true / nav:=true opt-in)

alias autodrive-autonomy='ros2 launch autodrive_neoracer autonomy.launch.py'

# AutoDRIVE's equivalent of pressing START on the gamepad

alias press-start='ros2 topic pub --times 6 -r 2 /joy sensor_msgs/msg/Joy "{axes: [0,0,0,0,0,0,0,0], buttons: [0,0,0,0,0,0,0,1,0,0,0]}" >/dev/null 2>&1; ros2 topic pub --times 2 -r 2 /joy sensor_msgs/msg/Joy "{axes: [0,0,0,0,0,0,0,0], buttons: [0,0,0,0,0,0,0,0,0,0,0]}" >/dev/null 2>&1; echo "START pressed"'

# <<< NeoRacer AutoDRIVE <<<

# NeoRacer - shell tool

[ -f "\$HOME/ros2_ws/src/neoracer_ros2_driver/scripts/racecar-tool.sh" ] && \
    source "\$HOME/ros2_ws/src/neoracer_ros2_driver/scripts/racecar-tool.sh"
BLOCK
fi
USEREOF

################################################################################

echo ">> [4/6] Installing JupyterLab and AI Stack (User-Level)"

sudo -u "$TARGET_USER" pip3 install --user "${PIP_FLAGS[@]}" --quiet jupyterlab
sudo -u "$TARGET_USER" pip3 install --user "${PIP_FLAGS[@]}" --quiet ultralytics "numpy<2"
sudo -u "$TARGET_USER" pip3 install --user "${PIP_FLAGS[@]}" --quiet pandas
sudo -u "$TARGET_USER" pip3 install --user "${PIP_FLAGS[@]}" --quiet scipy==1.14

################################################################################

echo ">> [5/6] Setting Up Lab Dashboards"

GITHUB_ORG=https://github.com/Neobotics-Foundation-Inc
DASHBOARDS_DIR="$HOME_DIR/neoracer_ros2_driver/scripts/dashboards"
DASHBOARDS=(
    camlabel:camlabel_dashboard
    wallfollow:wallfollow_dashboard
    pursuit:pursuit_dashboard
    eps:eps_dashboard
    smartfollow:smartfollow_dashboard
    linefollow:linefollow_dashboard
    webteleop:teleop_dashboard
)

sudo -u "$TARGET_USER" mkdir -p "$DASHBOARDS_DIR"
for entry in "${DASHBOARDS[@]}"; do
    name="${entry%%:*}" dir="$DASHBOARDS_DIR/${entry#*:}"
    if [ ! -d "$dir/.git" ]; then
        sudo -u "$TARGET_USER" git clone -q "$GITHUB_ORG/${entry#*:}.git" "$dir" 2>/dev/null \
            && echo "  $name: cloned" \
            || echo "  $name: clone failed; skipped (needs internet)" >&2
    else
        sudo -u "$TARGET_USER" git -C "$dir" pull -q --ff-only 2>/dev/null \
            && echo "  $name: at origin tip" \
            || echo "  $name: not fast-forwardable; left as is" >&2
    fi
done

################################################################################

echo ">> [6/6] Service manager"
# Services replicate the physical car's operational surface. systemd is used
# on a normal Ubuntu install; Docker uses the supervisord backend below. Each
# unit still wraps one unchanged foreground command.
if [ -d /run/systemd/system ]; then
    bash "$SCRIPT_DIR/autodrive_services.sh"
else
    bash "$SCRIPT_DIR/autodrive_supervisor.sh"
fi

echo "Done."
# Inside a container, hostname -I reports the container's internal address,
# which the AutoDRIVE host cannot reach; the docker-published port on the
# host is the real endpoint.
if [ -d /run/systemd/system ]; then
    IP=$(hostname -I | awk '{print $1}')
else
    IP="the Docker host's address (port 4567 is published; localhost works on the same machine)"
fi
cat <<DONE
==============================================================================
 AutoDRIVE-NeoRacer Development Environment Ready!

 1) AutoDRIVE Simulator (Windows host): open the AutoDRIVE project in Unity,
    select scene 'NeoRacer - Test', press Play, then
    Connect to  $IP : 4567  and set Driving Mode: Autonomous.
    (Native Ubuntu: use 127.0.0.1)

 2) AutoDRIVE:  ~/start_autodrive_bridge.sh (bridge + real mux/throttle)
 3) Autonomy:   ~/run_autodrive_lab.sh      (racecar_core lab; any lab from
                ~/jupyter_ws/neoracer-os/labs works, e.g. lab_e wall follower)
==============================================================================
DONE

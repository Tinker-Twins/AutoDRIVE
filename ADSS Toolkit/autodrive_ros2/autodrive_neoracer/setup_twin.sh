#!/bin/bash
# One-shot setup for the NeoRacer digital twin ROS 2 environment.
# Targets Ubuntu 22.04 + ROS 2 Humble - the same distro as the car. Usage:
#   sudo bash setup_twin.sh
set -e
if [ "$EUID" -ne 0 ]; then echo "Run with: sudo bash setup_twin.sh"; exit 1; fi
# id -un rather than $USER: docker exec shells have no login environment,
# so $USER is unset there.
TARGET_USER="${SUDO_USER:-$(id -un)}"
HOME_DIR=$(eval echo "~$TARGET_USER")
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# Containers ship without a configured tzdata; without this, apt stops at an
# interactive timezone prompt.
export DEBIAN_FRONTEND=noninteractive
# PEP 668 flag exists only on Ubuntu >= 23.04 pips
PIP_FLAGS=()
pip3 install --help 2>/dev/null | grep -q break-system-packages && PIP_FLAGS+=(--break-system-packages)

echo ">> [1/4] ROS 2 Humble"
apt-get update -qq
apt-get install -y -qq curl gnupg lsb-release software-properties-common git sudo
add-apt-repository -y universe >/dev/null 2>&1
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
printf 'deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu %s main\n' \
  "$(lsb_release -cs)" > /etc/apt/sources.list.d/ros2.list
apt-get update -qq
# catkin-pkg ships in both Ubuntu and ROS repos; force-overwrite resolves the file collision
apt-get install -y -o Dpkg::Options::=--force-overwrite python3-catkin-pkg-modules
apt-get install -y ros-humble-ros-base ros-humble-ackermann-msgs ros-humble-cv-bridge \
  python3-colcon-common-extensions python3-opencv python3-pip \
  ros-humble-robot-localization ros-humble-imu-complementary-filter \
  ros-humble-slam-toolbox ros-humble-nav2-bringup ros-humble-joint-state-publisher \
  ros-humble-diagnostic-updater ros-humble-vision-msgs

echo ">> [2/4] Python dependencies"
# socketio/engineio pins: the simulator speaks Socket.IO protocol EIO=3.
pip3 install "${PIP_FLAGS[@]}" \
  "python-socketio==4.2.0" "python-engineio==3.13.0" gevent gevent-websocket \
  numpy luma.led_matrix

echo ">> [3/4] User environment"
sudo -u "$TARGET_USER" bash <<USEREOF
set -e
cd "$HOME_DIR"
# FastDDS UDP-only profile: prevents one-way silent failures caused by stale or
# cross-user shared-memory segments (especially on WSL2)
cp "$SCRIPT_DIR/fastdds_udp_only.xml" "$HOME_DIR/fastdds_udp_only.xml"
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
cp "$SCRIPT_DIR/labs/twin_demo_lab.py" "\$NEO_OS/labs/"
# Twin-only: the wall follower's RB deadman guards fragile hardware; the sim
# has none, so disable it in THIS copy. The neoracer-labs repo default stays True.
[ -f "\$NEO_OS/labs/ultimate-wall-follower/wall_follower.py" ] && \
  sed -i "s/^REQUIRE_DEADMAN = True/REQUIRE_DEADMAN = False/" \
    "\$NEO_OS/labs/ultimate-wall-follower/wall_follower.py"
SITE=\$(python3 -c "import site; print(site.getusersitepackages())")
mkdir -p "\$SITE"
echo "\$NEO_OS/library" > "\$SITE/racecar_student.pth"
# ROS 2 workspace: the twin bridge plus the REAL driver package, whose mux and
# throttle nodes run unmodified in the twin (sim_twin.launch.py starts them).
mkdir -p ros2_ws/src
# This script ships in two layouts: the simulator repo's Devkit/ (package in
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
# osrbot has deleted branches under us before (dev, 2026-08), and the twin
# must build the tree it was validated against. Bump deliberately.
OSRACER_PIN=378947f471c70f2c78f150109837fa0787820115
mkdir -p osracer_ws/src
[ -d osracer_ws/src/osracer ] || \
  git clone -q https://github.com/osrbot/osracer.git osracer_ws/src/osracer
git -C osracer_ws/src/osracer checkout -q "\$OSRACER_PIN"
cd osracer_ws && colcon build --symlink-install \
  --packages-select osracer_description osracer_bringup osracer_slam osracer_navigation \
                    osracer_debug osracer_calib
cd "$HOME_DIR"
# launchers (fallbacks; the canonical workflow is the bashrc aliases below)
cp "$SCRIPT_DIR/start_twin_bridge.sh" "$HOME_DIR/start_twin_bridge.sh"
cp "$SCRIPT_DIR/run_twin_lab.sh" "$HOME_DIR/run_twin_lab.sh"
chmod +x "$HOME_DIR/start_twin_bridge.sh" "$HOME_DIR/run_twin_lab.sh"
# Jetson-parity shell environment: same structure as the real NeoRacer, where
# .bashrc sources ROS + workspace + library and 'teleop' is a launch alias.
if ! grep -q "NeoRacer digital twin" "$HOME_DIR/.bashrc"; then
cat >> "$HOME_DIR/.bashrc" <<'BLOCK'

# >>> NeoRacer digital twin (mirrors the Jetson bashrc) >>>
source /opt/ros/humble/setup.bash
# osracer underlay BENEATH the neoracer overlay, same order as the car's
# launch_autonomy.sh
# NOTE: this BLOCK sits inside the outer USEREOF heredoc, which expands
# variables as root - every literal-$HOME below must stay escaped.
[ -f \$HOME/osracer_ws/install/setup.bash ] && source \$HOME/osracer_ws/install/setup.bash
source \$HOME/ros2_ws/install/setup.bash
export FASTRTPS_DEFAULT_PROFILES_FILE=\$HOME/fastdds_udp_only.xml
# 'teleop' on the car brings up the driver; 'twin' brings up bridge + mux + throttle
alias twin='ros2 launch autodrive_neoracer sim_twin.launch.py'
# the car's autonomy graph with ground-truth TF (slam:=true / nav:=true opt-in)
alias twin-autonomy='ros2 launch autodrive_neoracer twin_autonomy.launch.py'
# the twin's equivalent of pressing START on the gamepad
alias press-start='ros2 topic pub --times 6 -r 2 /joy sensor_msgs/msg/Joy "{axes: [0,0,0,0,0,0,0,0], buttons: [0,0,0,0,0,0,0,1,0,0,0]}" >/dev/null 2>&1; ros2 topic pub --times 2 -r 2 /joy sensor_msgs/msg/Joy "{axes: [0,0,0,0,0,0,0,0], buttons: [0,0,0,0,0,0,0,0,0,0,0]}" >/dev/null 2>&1; echo "START pressed"'
# <<< NeoRacer digital twin <<<

# Neoracer - shell tool
[ -f "\$HOME/ros2_ws/src/neoracer_ros2_driver/scripts/racecar-tool.sh" ] && \
    source "\$HOME/ros2_ws/src/neoracer_ros2_driver/scripts/racecar-tool.sh"
BLOCK
fi
USEREOF

echo ">> [4/5] JupyterLab + ML stack (user-level)"
sudo -u "$TARGET_USER" pip3 install --user "${PIP_FLAGS[@]}" --quiet jupyterlab
# ultralytics for the driver's YOLO node (multi-GB torch pull). Its numpy 2
# breaks the apt matplotlib in the same interpreter, and pip skips a plain
# --user matplotlib as already-satisfied, so force the user-site copy.
sudo -u "$TARGET_USER" pip3 install --user "${PIP_FLAGS[@]}" --quiet ultralytics
sudo -u "$TARGET_USER" pip3 install --user "${PIP_FLAGS[@]}" --ignore-installed --quiet matplotlib

echo ">> [5/6] Lab dashboards (content)"
# Cloned regardless of init system: each dashboard is a plain Python HTTP
# server that can run foreground. Unit installation is a separate, systemd-
# only concern (setup_twin_services.sh).
GITHUB_ORG=https://github.com/Neobotics-Foundation-Inc
DASHBOARDS_DIR="$HOME_DIR/neoracer_ros2_driver/scripts/dashboards"
DASHBOARDS=(
    camlabel:camlabel_dashboard
    wallfollow:wallfollow_dashboard
    pursuit:pursuit_dashboard
    eps:eps_dashboard
    smartfollow:smartfollow_dashboard
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

echo ">> [6/6] systemd services (racecar service parity)"
# Services replicate the physical car's operational surface and need systemd
# as the running init. In a container there is no init to register with, so
# the same components run foreground instead; see SETUP.md, 'Running in a
# container'. Functionality is identical: every unit only wraps one of these
# foreground commands.
if [ -d /run/systemd/system ]; then
    bash "$SCRIPT_DIR/setup_twin_services.sh"
else
    echo "systemd is not the running init (container?); skipping service"
    echo "installation. Run the stack foreground per SETUP.md:"
    echo "  ros2 launch autodrive_neoracer sim_twin.launch.py"
    echo "  ros2 launch autodrive_neoracer twin_autonomy.launch.py"
fi

echo "Done."
IP=$(hostname -I | awk '{print $1}')
cat <<DONE
=========================================================
 NeoRacer digital twin environment ready.

 1) Simulator (Windows host): open the AutoDRIVE-Sim Unity
    project, scene 'NeoRacer - Test', press Play, then
    Connect to  $IP : 4567  and set Driving Mode: Autonomous.
    (Native Ubuntu: use 127.0.0.1)

 2) Twin stack: ~/start_twin_bridge.sh   (bridge + real mux/throttle)
 3) Autonomy:   ~/run_twin_lab.sh        (racecar_core lab; any lab from
                ~/jupyter_ws/neoracer-os/labs works, e.g. lab_e wall follower)

 Optional, for every shell to default to UDP-only DDS:
   echo 'export FASTRTPS_DEFAULT_PROFILES_FILE=\$HOME/fastdds_udp_only.xml' >> ~/.bashrc
=========================================================
DONE

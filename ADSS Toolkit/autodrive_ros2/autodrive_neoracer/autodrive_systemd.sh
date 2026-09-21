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

# AutoDRIVE Devkit Services Script for NeoRacer (systemd)
#
# This script sets up the AutoDRIVE-NeoRacer systemd services on Ubuntu 22.04
# with ROS 2 Humble substrate. The systemd unit names are unchanged, so the
# digital twin exercises same behavior as the physical twin of the vehicle:
# racecar service {status,start,stop,restart,logs}
#
# Usage: sudo bash autodrive_systemd.sh

#################################################################################

set -e

if [ "$EUID" -ne 0 ]; then echo "Please run as root: sudo bash autodrive_systemd.sh"; exit 1; fi
TARGET_USER="${SUDO_USER:-$(id -un)}"
HOME_DIR=$(eval echo "~$TARGET_USER")

ros_env="source /opt/ros/humble/setup.bash && \\
    [ -f $HOME_DIR/osracer_ws/install/setup.bash ] && source $HOME_DIR/osracer_ws/install/setup.bash; \\
    source $HOME_DIR/ros2_ws/install/setup.bash && \\
    export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME_DIR/fastdds_udp.xml"

unit() {
    local name="$1" desc="$2" exec_cmd="$3" extra="$4"
    cat > "/etc/systemd/system/neoracer-$name.service" <<UNIT
[Unit]
Description=$desc
$extra

[Service]
Type=exec
User=$TARGET_USER
Group=$TARGET_USER
ExecStart=/bin/bash -c '$ros_env && exec $exec_cmd'
Restart=on-failure
RestartSec=5
Environment=HOME=$HOME_DIR
WorkingDirectory=$HOME_DIR

[Install]
WantedBy=multi-user.target
UNIT
}

unit teleop "AutoDRIVE-NeoRacer Bringup" \
    "ros2 launch autodrive_neoracer autodrive.launch.py" \
    "After=network-online.target"

unit autonomy "AutoDRIVE-NeoRacer Autonomy" \
    "ros2 launch autodrive_neoracer autonomy.launch.py" \
    "After=neoracer-teleop.service"

unit dashboard "AutoDRIVE-NeoRacer System Dashboard" \
    "python3 $HOME_DIR/ros2_ws/src/neoracer_ros2_driver/scripts/dashboard.py" \
    "After=neoracer-teleop.service"

unit watchdog "AutoDRIVE-NeoRacer Watchdog" \
    "ros2 run autodrive_neoracer autodrive_watchdog" \
    "After=neoracer-teleop.service"

unit jupyter "AutoDRIVE-NeoRacer JupyterLab" \
    "$HOME_DIR/.local/bin/jupyter-lab --ip=0.0.0.0 --port=8888 --no-browser \\
        --notebook-dir=$HOME_DIR/jupyter_ws --ServerApp.token=\"\" --ServerApp.password=\"\"" \
    "After=network-online.target"

systemctl daemon-reload
systemctl enable neoracer-teleop neoracer-autonomy neoracer-dashboard neoracer-jupyter neoracer-watchdog
systemctl restart neoracer-teleop neoracer-autonomy neoracer-dashboard neoracer-jupyter neoracer-watchdog

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
echo
echo "Lab Dashboards (installed but disabled):"
for entry in "${DASHBOARDS[@]}"; do
    name="${entry%%:*}" dir="$DASHBOARDS_DIR/${entry#*:}"
    [[ -d "$dir" ]] || { echo "  $name: checkout missing; run autodrive_install.sh" >&2; continue; }
    bash "$dir/setup.sh" || { echo "  $name: setup.sh failed" >&2; continue; }
    py=$(ls "$dir"/*.py 2>/dev/null | head -1)
    mkdir -p "/etc/systemd/system/neoracer-$name.service.d"
    cat > "/etc/systemd/system/neoracer-$name.service.d/autodrive.conf" <<DROPIN
[Service]
User=$TARGET_USER
Group=$TARGET_USER
Environment=HOME=$HOME_DIR
Environment=FASTRTPS_DEFAULT_PROFILES_FILE=$HOME_DIR/fastdds_udp.xml
ExecStart=/bin/bash -c "source /opt/ros/humble/setup.bash && source $HOME_DIR/ros2_ws/install/setup.bash && exec python3 $py"
DROPIN
done
systemctl daemon-reload
echo
systemctl --no-pager --no-legend list-units 'neoracer-*' || true

################################################################################

echo "AutoDRIVE-NeoRacer Systemd Services Ready!"

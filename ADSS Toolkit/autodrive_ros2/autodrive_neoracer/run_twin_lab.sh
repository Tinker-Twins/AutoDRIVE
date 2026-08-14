#!/bin/bash
# Run a racecar_core lab against the NeoRacer digital twin.
# Usage: ./run_twin_lab.sh [lab_file.py]   (default: twin_demo_lab.py)
source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/install/setup.bash"
export FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/fastdds_udp_only.xml"
# The library resolves through racecar_student.pth in the user site-packages
# (same selection mechanism as `racecar library --select` on the car).
LAB="${1:-$HOME/jupyter_ws/neoracer-os/labs/twin_demo_lab.py}"

# START is pressed automatically by the bridge's virtual gamepad when the lab
# attaches; no manual press needed.
# System interpreter explicitly: rclpy's C extensions are built for it, and an
# active venv must not shadow it.
exec /usr/bin/python3 "$LAB" -h

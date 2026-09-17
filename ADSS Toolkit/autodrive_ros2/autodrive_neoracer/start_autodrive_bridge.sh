#!/bin/bash
# Start the NeoRacer AutoDRIVE stack: Socket.IO bridge (:4567) plus the
# real driver's mux and throttle nodes, so /drive flows through the same
# chain as on hardware.
source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/install/setup.bash"
export FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/fastdds_udp.xml"
exec ros2 launch autodrive_neoracer autodrive.launch.py

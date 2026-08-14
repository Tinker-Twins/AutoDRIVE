#!/bin/bash
# Install the car's neoracer systemd services adapted for the twin.
# Same unit names, so `racecar service status/start/stop/restart/logs` works
# exactly as on the car. Usage:
#   sudo bash setup_twin_services.sh
#
# Twin adaptations vs the car's scripts/neoracer-*.service:
#   - teleop runs the twin stack (bridge + real mux/throttle) instead of the
#     hardware drivers; on the twin the simulator IS the hardware.
#   - ROS 2 Jazzy paths and the invoking user instead of humble//home/racecar.
#   - watchdog is installed but left disabled: it pgreps for the hardware
#     controller and lakibeam processes, which never run in the twin, so it
#     would restart-loop teleop. Enable it only if that ever changes.
set -e
if [ "$EUID" -ne 0 ]; then echo "Run with: sudo bash setup_twin_services.sh"; exit 1; fi
TARGET_USER="${SUDO_USER:-$(id -un)}"
HOME_DIR=$(eval echo "~$TARGET_USER")

ros_env="source /opt/ros/humble/setup.bash && \\
    [ -f $HOME_DIR/osracer_ws/install/setup.bash ] && source $HOME_DIR/osracer_ws/install/setup.bash; \\
    source $HOME_DIR/ros2_ws/install/setup.bash && \\
    export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME_DIR/fastdds_udp_only.xml"

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

unit teleop "Neoracer Twin Stack (bridge :4567 + mux + throttle)" \
    "ros2 launch autodrive_neoracer sim_twin.launch.py" \
    "After=network-online.target"

unit autonomy "Neoracer Autonomy Base (ground-truth TF + EKF + twist bridge)" \
    "ros2 launch autodrive_neoracer twin_autonomy.launch.py" \
    "After=neoracer-teleop.service"

unit dashboard "Neoracer Web Dashboard (port 8080)" \
    "python3 $HOME_DIR/ros2_ws/src/neoracer_ros2_driver/scripts/dashboard.py" \
    "After=neoracer-teleop.service"

unit watchdog "Neoracer Node Watchdog (DISABLED on twin: pgreps hardware processes)" \
    "python3 $HOME_DIR/ros2_ws/src/neoracer_ros2_driver/scripts/watchdog.py" \
    "After=neoracer-teleop.service
BindsTo=neoracer-teleop.service"

unit jupyter "Neoracer JupyterLab (port 8888)" \
    "$HOME_DIR/.local/bin/jupyter-lab --ip=0.0.0.0 --port=8888 --no-browser \\
        --notebook-dir=$HOME_DIR/jupyter_ws --ServerApp.token=\"\" --ServerApp.password=\"\"" \
    "After=network-online.target"

systemctl daemon-reload
systemctl enable neoracer-teleop neoracer-autonomy neoracer-dashboard neoracer-jupyter
systemctl restart neoracer-teleop neoracer-autonomy neoracer-dashboard neoracer-jupyter
systemctl disable --now neoracer-watchdog 2>/dev/null || true

# Lab dashboard units. The checkouts themselves are content, cloned by
# setup_twin.sh regardless of init system; this installs each one's unit
# (its own setup.sh leaves it stopped and disabled), matching the car:
# `racecar service` starts one when a lab needs it.
DASHBOARDS_DIR="$HOME_DIR/neoracer_ros2_driver/scripts/dashboards"
DASHBOARDS=(
    camlabel:camlabel_dashboard
    wallfollow:wallfollow_dashboard
    pursuit:pursuit_dashboard
    eps:eps_dashboard
    smartfollow:smartfollow_dashboard
)
echo
echo "Lab dashboards (installed disabled, like the car):"
for entry in "${DASHBOARDS[@]}"; do
    name="${entry%%:*}" dir="$DASHBOARDS_DIR/${entry#*:}"
    [[ -d "$dir" ]] || { echo "  $name: checkout missing; run setup_twin.sh" >&2; continue; }
    bash "$dir/setup.sh" || { echo "  $name: setup.sh failed" >&2; continue; }
    # The dashboards' setup.sh renders units for the factory user (racecar)
    # and its home paths. Override user, HOME, and the workspace source in a
    # drop-in; the base unit can then be re-rendered by setup.sh at any time
    # without losing the twin adaptation.
    py=$(ls "$dir"/*.py 2>/dev/null | head -1)
    mkdir -p "/etc/systemd/system/neoracer-$name.service.d"
    cat > "/etc/systemd/system/neoracer-$name.service.d/twin.conf" <<DROPIN
[Service]
User=$TARGET_USER
Group=$TARGET_USER
Environment=HOME=$HOME_DIR
Environment=FASTRTPS_DEFAULT_PROFILES_FILE=$HOME_DIR/fastdds_udp_only.xml
ExecStart=
ExecStart=/bin/bash -c "source /opt/ros/humble/setup.bash && source $HOME_DIR/ros2_ws/install/setup.bash && exec python3 $py"
DROPIN
done
systemctl daemon-reload

echo
systemctl --no-pager --no-legend list-units 'neoracer-*' || true
cat <<DONE
=========================================================
 Twin services installed. Manage them like the car:
   racecar service status
   racecar service restart [teleop|autonomy|dashboard|jupyter]
   racecar service logs teleop
 Dashboard: http://localhost:8080   JupyterLab: http://localhost:8888
 NOTE: do not run 'racecar service install' on the twin - it installs the
 car's hardware units. Use this script instead.
=========================================================
DONE

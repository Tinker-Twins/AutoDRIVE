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

# AutoDRIVE Devkit Services Script for NeoRacer
#
# This script sets up the AutoDRIVE-NeoRacer supervisord services on Ubuntu
# 22.04 with ROS 2 Humble substrate. The service names are unchanged, so the
# digital twin uses the same command interface as the physical twin:
# racecar service {status,start,stop,restart,logs}
#
# Usage: sudo bash autodrive_services.sh

################################################################################

set -e

# Require root and resolve the script location, service user, and workspace paths.
[ "$EUID" -eq 0 ] || exit 1
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TARGET_USER="${SUDO_USER:-$(id -un)}"
HOME_DIR=$(eval echo "~$TARGET_USER")
DRIVER="$HOME_DIR/ros2_ws/src/neoracer_ros2_driver"
STATE=/var/lib/neoracer-supervisor
CONF=/etc/supervisor/conf.d/neoracer.conf

#########################################################
# SERVICE STARTUP PREFERENCES
#########################################################

# Runtime intent and recovery lock shared with the watchdog (not boot preferences).
mkdir -p "$STATE" /etc/supervisor/conf.d
mkdir -p /run/neoracer-supervisor
chown "$TARGET_USER" /run/neoracer-supervisor
touch /run/neoracer-supervisor/teleop.lock
chown "$TARGET_USER" /run/neoracer-supervisor/teleop.lock
chmod 600 /run/neoracer-supervisor/teleop.lock
printf '%s\n' teleop watchdog dashboard jupyter > "$STATE/enabled"
: > "$STATE/disabled"

#########################################################
# SUPERVISOR AND CORE SERVICES
#########################################################

# Write the manager configuration and core service definitions.
# Registered programs start with Supervisor; automatic process restarts are disabled.
# The startup helper also attempts to start services in the saved enabled list.
# Each service saves stdout in its own log; stderr is not saved.
cat > "$CONF" <<CONF
# Local control socket shared by supervisord and supervisorctl.
[unix_http_server]
file=/run/supervisor.sock

# Run the manager in the background and record its log and process ID.
[supervisord]
logfile=/var/log/supervisord.log
pidfile=/run/supervisord.pid
nodaemon=false

# Connect the command-line client to the manager through the local socket.
[supervisorctl]
serverurl=unix:///run/supervisor.sock

# Enable the XML-RPC interface used by supervisorctl.
[rpcinterface:supervisor]
supervisor.rpcinterface_factory = supervisor.rpcinterface:make_main_rpcinterface

# Teleop: load ROS and launch the AutoDRIVE vehicle interface.
[program:neoracer-teleop]
command=/bin/bash -lc 'source /opt/ros/humble/setup.bash && source $HOME_DIR/ros2_ws/install/setup.bash && export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME_DIR/fastdds_udp.xml && exec ros2 launch autodrive_neoracer autodrive.launch.py'
directory=$HOME_DIR
user=$TARGET_USER
autostart=false
autorestart=false
# Stop the launcher and its ROS children together.
stopasgroup=true
killasgroup=true
stdout_logfile=/var/log/neoracer-teleop.log
stderr_logfile=NONE

# Autonomy: also load the navigation workspace before launching the autonomy stack.
[program:neoracer-autonomy]
command=/bin/bash -lc 'source /opt/ros/humble/setup.bash && source $HOME_DIR/osracer_ws/install/setup.bash && source $HOME_DIR/ros2_ws/install/setup.bash && export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME_DIR/fastdds_udp.xml && exec ros2 launch autodrive_neoracer autonomy.launch.py'
directory=$HOME_DIR
user=$TARGET_USER
autostart=false
autorestart=unexpected
stdout_logfile=/var/log/neoracer-autonomy.log
stderr_logfile=NONE

# Dashboard: run the NeoRacer monitoring and service-control interface.
[program:neoracer-dashboard]
command=/bin/bash -lc 'source /opt/ros/humble/setup.bash && source $HOME_DIR/ros2_ws/install/setup.bash && exec python3 $HOME_DIR/ros2_ws/src/neoracer_ros2_driver/scripts/dashboard.py'
directory=$HOME_DIR
user=$TARGET_USER
autostart=false
autorestart=unexpected
stdout_logfile=/var/log/neoracer-dashboard.log
stderr_logfile=NONE

# JupyterLab: serve notebooks on port 8888 with authentication disabled.
[program:neoracer-jupyter]
command=/bin/bash -lc 'exec $HOME_DIR/.local/bin/jupyter-lab --allow-root --ip=0.0.0.0 --port=8888 --no-browser --notebook-dir=$HOME_DIR/jupyter_ws --ServerApp.token="" --ServerApp.password=""'
directory=$HOME_DIR
user=$TARGET_USER
autostart=false
autorestart=unexpected
stdout_logfile=/var/log/neoracer-jupyter.log
stderr_logfile=NONE
CONF

#########################################################
# WATCHDOG SERVICE
#########################################################

# Append the AutoDRIVE Watchdog, which monitors the vehicle stack and performs recovery.
cat >> "$CONF" <<CONF

[program:neoracer-watchdog]
command=/bin/bash -lc 'source /opt/ros/humble/setup.bash && source $HOME_DIR/ros2_ws/install/setup.bash && exec ros2 run autodrive_neoracer autodrive_watchdog'
directory=$HOME_DIR
user=$TARGET_USER
autostart=false
autorestart=true
# Stop the launcher and its ROS children together.
stopasgroup=true
killasgroup=true
stdout_logfile=/var/log/neoracer-watchdog.log
stderr_logfile=NONE
CONF

#########################################################
# LAB DASHBOARD SERVICES
#########################################################

# Map optional lab service names to their dashboard entry points.
DASHBOARDS_DIR="$HOME_DIR/neoracer_ros2_driver/scripts/dashboards"
declare -A DASH=(
  [camlabel]="$DASHBOARDS_DIR/camlabel_dashboard/camlabel.py"
  [wallfollow]="$DASHBOARDS_DIR/wallfollow_dashboard/wallfollow.py"
  [pursuit]="$DASHBOARDS_DIR/pursuit_dashboard/pursuit.py"
  [eps]="$DASHBOARDS_DIR/eps_dashboard/eps.py"
  [smartfollow]="$DASHBOARDS_DIR/smartfollow_dashboard/smartfollow.py"
  [linefollow]="$DASHBOARDS_DIR/linefollow_dashboard/linefollow.py"
  [webteleop]="$DASHBOARDS_DIR/teleop_dashboard/teleop.py"
)

# Register only dashboards whose Python files exist; do not start them here.
for name in "${!DASH[@]}"; do
  py="${DASH[$name]}"
  [ -f "$py" ] || continue
  cat >> "$CONF" <<CONF

[program:neoracer-$name]
command=/bin/bash -lc 'source /opt/ros/humble/setup.bash && source $HOME_DIR/ros2_ws/install/setup.bash && exec /usr/bin/python3 $py'
directory=$HOME_DIR
user=$TARGET_USER
autostart=false
autorestart=true
stdout_logfile=/var/log/neoracer-$name.log
stderr_logfile=NONE
CONF
done

#########################################################
# CONTAINER STARTUP HELPER
#########################################################

# Install the startup helper invoked by the container entrypoint.
cat > /usr/local/sbin/neoracer-supervisor-start <<'START'
#!/bin/bash
set -e

# Start Supervisor and allow up to five seconds for its control socket to appear.
# A fresh container/service-manager startup begins a new manual-stop session.
rm -f /run/neoracer-supervisor/teleop.manually-stopped
supervisord -c /etc/supervisor/conf.d/neoracer.conf
for attempt in $(seq 1 50); do
  [ -S /run/supervisor.sock ] && break
  sleep 0.1
done

# Start saved enabled services; a failed start does not prevent later attempts.
for unit in $(cat /var/lib/neoracer-supervisor/enabled 2>/dev/null); do
  supervisorctl -c /etc/supervisor/conf.d/neoracer.conf start "neoracer-$unit" 2>/dev/null || true
done
START
chmod +x /usr/local/sbin/neoracer-supervisor-start

#########################################################
# RACECAR CLI INTEGRATION
#########################################################

# Preserve the racecar CLI by redirecting setup and service-management commands.
if [ -f "$DRIVER/scripts/racecar-tool.sh" ]; then
    if [ -f "$SCRIPT_DIR/autodrive_services.sh" ] && [[ "$SCRIPT_DIR" == */autodrive_devkit ]]; then
        cp "$SCRIPT_DIR/autodrive_services.sh" "$HOME_DIR/autodrive_services.sh"
    fi

    # Update both upstream setup commands and previously installed script names.
    sed -i 's#bash "$pkg_dir/scripts/setup_services.sh"#bash "$HOME/autodrive_services.sh"#; s#bash "$HOME/autodrive_neoracer_setup_services.sh"#bash "$HOME/autodrive_services.sh"#; s#bash "$HOME/autodrive_supervisor.sh"#bash "$HOME/autodrive_services.sh"#; s#bash "$HOME/autodrive_supervisord.sh"#bash "$HOME/autodrive_services.sh"#' "$DRIVER/scripts/racecar-tool.sh"

    # Mask systemctl commands and trigger a wrapper script or a specific
    # custom version of those system utilities instead for safer operation.
    sed -i 's#sudo systemctl#sudo /usr/local/bin/systemctl#g; s#sudo journalctl#sudo /usr/local/bin/journalctl#g' "$DRIVER/scripts/racecar-tool.sh"
fi

#########################################################
# SYSTEMCTL COMPATIBILITY
#########################################################

# Install a limited systemctl compatibility wrapper backed by supervisorctl.
cat > /usr/local/bin/systemctl <<'SYSTEMCTL'
#!/bin/bash
set -e
STATE=/var/lib/neoracer-supervisor
CTL=(supervisorctl -c /etc/supervisor/conf.d/neoracer.conf)

# Read the operation name, leaving service names in the remaining arguments.
action="${1:-}"; shift || true

case "$action" in
  # Accept reload requests without reloading Supervisor.
  daemon-reload) exit 0 ;;

  # List known service names, including optional services not installed.
  list-unit-files) printf '%s\n' neoracer-{teleop,watchdog,dashboard,jupyter,autonomy,camlabel,wallfollow,pursuit,eps,smartfollow,linefollow,webteleop}.service ;;

  # Report current process state independently of the saved startup preference.
  is-active)
    name="${1%.service}"
    if "${CTL[@]}" status "$name" 2>/dev/null | grep -q RUNNING; then
      echo active
    else
      echo inactive
      exit 3
    fi
    ;;

  # Report whether the startup helper is configured to start this service.
  is-enabled)
    name="${1#neoracer-}"
    if grep -qx "$name" "$STATE/enabled" 2>/dev/null; then
      echo enabled
    else
      echo disabled
      exit 1
    fi
    ;;

  # Update the startup lists without starting or stopping the service.
  enable|disable) name="${1#neoracer-}"; sed -i "/^$name$/d" "$STATE/enabled" "$STATE/disabled" 2>/dev/null || true; echo "$name" >> "$STATE/$([ "$action" = enable ] && echo enabled || echo disabled)" ;;

  # Forward each requested service operation to Supervisor.
  start|stop|restart)
    for unit in "$@"; do
      name="${unit#neoracer-}"
      name="${name%.service}"
      if [ "$name" = teleop ]; then
        (
          # Mark stop intent before waiting for an in-progress recovery.
          # The shared lock serializes the final start/stop operations.
          RUNTIME=/run/neoracer-supervisor
          [ "$action" != stop ] || touch "$RUNTIME/teleop.manually-stopped"
          exec 9>> "$RUNTIME/teleop.lock"
          flock -x 9
          if [ "$action" = stop ]; then
            touch "$RUNTIME/teleop.manually-stopped"
          else
            rm -f "$RUNTIME/teleop.manually-stopped"
          fi
          "${CTL[@]}" "$action" "neoracer-$name" >/dev/null
        )
      else
        "${CTL[@]}" "$action" "neoracer-$name" >/dev/null
      fi
    done
    ;;

  # Reject operations that this compatibility wrapper does not implement.
  *) exit 1 ;;
esac
SYSTEMCTL
chmod +x /usr/local/bin/systemctl

#########################################################
# JOURNALCTL COMPATIBILITY
#########################################################

# Install a limited journalctl replacement that follows the selected stdout log.
cat > /usr/local/bin/journalctl <<'JOURNALCTL'
#!/bin/bash
# Default to teleop; read the -u service argument and ignore other options.
unit=teleop
while [ "$#" -gt 0 ]; do
  [ "$1" = -u ] && { unit="${2#neoracer-}"; shift 2; continue; }
  shift
done

# Show the latest log lines and continue displaying new output.
tail -f "/var/log/neoracer-$unit.log"
JOURNALCTL
chmod +x /usr/local/bin/journalctl

################################################################################

echo "AutoDRIVE-NeoRacer Supervisord Services Ready!"

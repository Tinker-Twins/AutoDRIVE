#!/bin/bash
set -e
[ "$EUID" -eq 0 ] || exit 1
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TARGET_USER="${SUDO_USER:-$(id -un)}"
HOME_DIR=$(eval echo "~$TARGET_USER")
DRIVER="$HOME_DIR/ros2_ws/src/neoracer_ros2_driver"
STATE=/var/lib/neoracer-supervisor
CONF=/etc/supervisor/conf.d/neoracer.conf
mkdir -p "$STATE" /etc/supervisor/conf.d
printf '%s\n' teleop watchdog dashboard jupyter > "$STATE/enabled"
: > "$STATE/disabled"

cat > "$CONF" <<CONF
[unix_http_server]
file=/run/supervisor.sock
[supervisord]
logfile=/var/log/supervisord.log
pidfile=/run/supervisord.pid
nodaemon=false
[supervisorctl]
serverurl=unix:///run/supervisor.sock
[rpcinterface:supervisor]
supervisor.rpcinterface_factory = supervisor.rpcinterface:make_main_rpcinterface
[program:neoracer-teleop]
command=/bin/bash -lc 'source /opt/ros/humble/setup.bash && source $HOME_DIR/ros2_ws/install/setup.bash && export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME_DIR/fastdds_udp.xml && exec ros2 launch autodrive_neoracer autodrive.launch.py'
directory=$HOME_DIR
user=$TARGET_USER
autostart=false
autorestart=true
stdout_logfile=/var/log/neoracer-teleop.log
stderr_logfile=NONE
[program:neoracer-autonomy]
command=/bin/bash -lc 'source /opt/ros/humble/setup.bash && source $HOME_DIR/osracer_ws/install/setup.bash && source $HOME_DIR/ros2_ws/install/setup.bash && export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME_DIR/fastdds_udp.xml && exec ros2 launch autodrive_neoracer autonomy.launch.py'
directory=$HOME_DIR
user=$TARGET_USER
autostart=false
autorestart=true
stdout_logfile=/var/log/neoracer-autonomy.log
stderr_logfile=NONE
[program:neoracer-dashboard]
command=/bin/bash -lc 'source /opt/ros/humble/setup.bash && source $HOME_DIR/ros2_ws/install/setup.bash && exec python3 $HOME_DIR/ros2_ws/src/neoracer_ros2_driver/scripts/dashboard.py'
directory=$HOME_DIR
user=$TARGET_USER
autostart=false
autorestart=true
stdout_logfile=/var/log/neoracer-dashboard.log
stderr_logfile=NONE
[program:neoracer-jupyter]
command=/bin/bash -lc 'exec $HOME_DIR/.local/bin/jupyter-lab --allow-root --ip=0.0.0.0 --port=8888 --no-browser --notebook-dir=$HOME_DIR/jupyter_ws --ServerApp.token="" --ServerApp.password=""'
directory=$HOME_DIR
user=$TARGET_USER
autostart=false
autorestart=true
stdout_logfile=/var/log/neoracer-jupyter.log
stderr_logfile=NONE
CONF

cat >> "$CONF" <<CONF

[program:neoracer-watchdog]
command=/bin/bash -lc 'source /opt/ros/humble/setup.bash && source $HOME_DIR/ros2_ws/install/setup.bash && exec ros2 run autodrive_neoracer autodrive_watchdog'
directory=$HOME_DIR
user=$TARGET_USER
autostart=false
autorestart=true
stdout_logfile=/var/log/neoracer-watchdog.log
stderr_logfile=NONE
CONF

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

cat > /usr/local/sbin/neoracer-supervisor-start <<'START'
#!/bin/bash
set -e
supervisord -c /etc/supervisor/conf.d/neoracer.conf
for attempt in $(seq 1 50); do
  [ -S /run/supervisor.sock ] && break
  sleep 0.1
done
for unit in $(cat /var/lib/neoracer-supervisor/enabled 2>/dev/null); do
  supervisorctl -c /etc/supervisor/conf.d/neoracer.conf start "neoracer-$unit" 2>/dev/null || true
done
START
chmod +x /usr/local/sbin/neoracer-supervisor-start

if [ -f "$DRIVER/scripts/racecar-tool.sh" ]; then
    if [ -f "$SCRIPT_DIR/autodrive_supervisor.sh" ] && [[ "$SCRIPT_DIR" == */autodrive_devkit ]]; then
        cp "$SCRIPT_DIR/autodrive_supervisor.sh" "$HOME_DIR/autodrive_supervisor.sh"
    fi
    sed -i 's#bash "$pkg_dir/scripts/setup_services.sh"#bash "$HOME/autodrive_supervisor.sh"#; s#bash "$HOME/autodrive_neoracer_setup_services.sh"#bash "$HOME/autodrive_supervisor.sh"#' "$DRIVER/scripts/racecar-tool.sh"
    # sudo may use a restricted PATH; use the container compatibility tools
    # explicitly so racecar service commands never reach a real systemctl.
    sed -i 's#sudo systemctl#sudo /usr/local/bin/systemctl#g; s#sudo journalctl#sudo /usr/local/bin/journalctl#g' "$DRIVER/scripts/racecar-tool.sh"
fi

cat > /usr/local/bin/systemctl <<'SYSTEMCTL'
#!/bin/bash
set -e
STATE=/var/lib/neoracer-supervisor
CTL=(supervisorctl -c /etc/supervisor/conf.d/neoracer.conf)
action="${1:-}"; shift || true
case "$action" in
  daemon-reload) exit 0 ;;
  list-unit-files) printf '%s\n' neoracer-{teleop,watchdog,dashboard,jupyter,autonomy,camlabel,wallfollow,pursuit,eps,smartfollow,linefollow,webteleop}.service ;;
  is-active)
    name="${1%.service}"
    if "${CTL[@]}" status "$name" 2>/dev/null | grep -q RUNNING; then
      echo active
    else
      echo inactive
      exit 3
    fi
    ;;
  is-enabled)
    name="${1#neoracer-}"
    if grep -qx "$name" "$STATE/enabled" 2>/dev/null; then
      echo enabled
    else
      echo disabled
      exit 1
    fi
    ;;
  enable|disable) name="${1#neoracer-}"; sed -i "/^$name$/d" "$STATE/enabled" "$STATE/disabled" 2>/dev/null || true; echo "$name" >> "$STATE/$([ "$action" = enable ] && echo enabled || echo disabled)" ;;
  start|stop|restart) for unit in "$@"; do name="${unit#neoracer-}"; name="${name%.service}"; "${CTL[@]}" "$action" "neoracer-$name" >/dev/null; done ;;
  *) exit 1 ;;
esac
SYSTEMCTL
chmod +x /usr/local/bin/systemctl

cat > /usr/local/bin/journalctl <<'JOURNALCTL'
#!/bin/bash
unit=teleop
while [ "$#" -gt 0 ]; do
  [ "$1" = -u ] && { unit="${2#neoracer-}"; shift 2; continue; }
  shift
done
tail -f "/var/log/neoracer-$unit.log"
JOURNALCTL
chmod +x /usr/local/bin/journalctl

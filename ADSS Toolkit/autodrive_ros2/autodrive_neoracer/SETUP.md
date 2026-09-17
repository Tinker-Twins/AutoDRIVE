# NeoRacer on AutoDRIVE — Setup

End state: the NeoRacer drives in AutoDRIVE Simulator, controlled by
unmodified racecar_core lab code through the same ROS 2 topic contract,
command pipeline (mux → throttle → motor), and QoS as the physical car.
Code written for AutoDRIVE runs on the real NeoRacer without changes.

## Requirements

- Windows 10/11 with WSL2 + Ubuntu 22.04 (`wsl --install -d Ubuntu-22.04`
  from an admin PowerShell), or a native Ubuntu 22.04 machine. 22.04 is
  deliberate: AutoDRIVE runs ROS 2 Humble, the same distro as the car, so the
  driver's tooling and launches work unmodified.
- The distro must be WSL **2**. Check `wsl -l -v`; convert a WSL1 distro with
  `wsl --set-version Ubuntu-22.04 2`. Native/WSL systemd is used outside
  Docker; the container uses supervisord instead.
- Unity 2022.3.52f1 (Windows host or the Ubuntu machine).
- git on both sides.
- ~10 GB disk: AutoDRIVE project ~6 GB, ROS 2 + Nav2 stack ~3 GB.

## Part 1 — AutoDRIVE Simulator

1. Clone the fork and check out the NeoRacer branch:

   ```
   git clone https://github.com/Neobotics-Foundation-Inc/AutoDRIVE.git AutoDRIVE
   cd AutoDRIVE
   git checkout neoracer-vehicle
   ```

2. Open the project in Unity 2022.3.52f1. The first import takes 15–30
   minutes.

3. Open the scene `Assets/Scenes/NeoRacer - Test.unity`.

4. Only after changing the NeoRacer FBX or materials: menu
   **AutoDRIVE → Rebuild NeoRacer (All)**. Never run it in Play mode.

## Part 2 — Ubuntu workspace

One script installs everything. In an Ubuntu terminal:

```
cd /mnt/c/<path-to>/AutoDRIVE/Devkit     # native Ubuntu: the checkout path
sudo bash autodrive_install.sh
```

What it does:

- Installs ROS 2 Humble plus `ackermann_msgs`, `cv_bridge`,
  `robot_localization`, `imu_complementary_filter`, `slam_toolbox`,
  `nav2_bringup`, `joint_state_publisher`.
- Pins the Python Socket.IO stack the AutoDRIVE protocol requires
  (`python-socketio==4.2.0`, `python-engineio==3.13.0`, gevent).
- Assembles the student library and labs from the Neobotics forks into
  `~/jupyter_ws/neoracer-os/{library,labs}` and selects the library through
  `racecar_student.pth` in the user site-packages — the same layout and
  selection mechanism the car's setup creates.
- Clones `neoracer_ros2_driver` and builds its mux and throttle nodes into
  `~/ros2_ws` next to the `autodrive_neoracer` bridge. AutoDRIVE runs the real
  driver nodes; the bridge only replaces hardware I/O.
- Clones the osracer vendor stack (dev branch — the factory-image layout)
  into `~/osracer_ws` and builds `osracer_description`, `osracer_bringup`,
  `osracer_slam`, `osracer_navigation`.
- Installs a FastDDS UDP-only profile (prevents WSL2 shared-memory
  transport failures) and appends the AutoDRIVE aliases to `~/.bashrc`.

Open a new terminal when it finishes so the aliases load.

If `apt` fails with 404 errors, the package index is stale:
`sudo apt update`, then rerun the script.

## Part 3 — Run

Setup installs the car-compatible services (same unit names, AutoDRIVE internals):
`neoracer-teleop` is the AutoDRIVE stack (bridge + real mux/throttle), plus
`neoracer-autonomy`, `neoracer-dashboard` (port 8080), and
`neoracer-jupyter` (port 8888). They start with the container. On Ubuntu they
are systemd units; in Docker they are supervisord programs, while the
`racecar service` interface stays unchanged:

```
racecar service status
racecar service restart              # all
racecar service restart teleop      # one
racecar service logs teleop
```

The AutoDRIVE watchdog is enabled and started by default. It monitors the
AutoDRIVE bridge, Foxglove bridge, mux, throttle, inference ROS node, and the
full NeoRacer topic contract listed below. It waits for AutoDRIVE data after
the AutoDRIVE connection on port `4567` before checking data-dependent health.
If the teleop service stops or a required runtime node is missing, it stops
teleop, releases TCP port `4567`, and restarts teleop only after the port is
confirmed closed. Critical topic names are also checked, but their absence is
reported only: `/drive` and `/gamepad_drive` are normally absent until a lab
or teleoperation client starts. If the port cannot be released, recovery is
aborted and the failure is written to
`~/logs/latest/watchdog.log`, which appears in the dashboard's **Watchdog log**.
Loss of AutoDRIVE data itself is reported there but does not trigger a restart.
`racecar service install` is also safe in the container and re-applies the
supervisord backend.

To run the stack in a foreground terminal instead (stop the service first):
`autodrive`, and `autodrive-autonomy` for the autonomy layer.

Find the Ubuntu IP: `hostname -I` (first address). Native Ubuntu:
`127.0.0.1`. The WSL2 address changes on reboot.

AutoDRIVE Simulator — press Play, set the IP and port `4567` in the left menu, set
**Driving Mode: Autonomous**, press **Connect**. The HUD turns green and
the teleop log (`racecar service logs teleop`) counts frames.

Lab code, any terminal:

```
python3 ~/jupyter_ws/neoracer-os/labs/disparity_extender.py -h
```

The bridge presses START automatically. Every lab in
`~/jupyter_ws/neoracer-os/labs/` (lab_a–lab_i, grand_prix, and the rest)
runs in AutoDRIVE the same way, or from JupyterLab at
`http://localhost:8888`. Dashboard: `http://localhost:8080`.

## Part 4 — Autonomy layer (optional)

The car's autonomy base — TF from `osracer_description`, IMU complementary
filter + EKF publishing `/odometry/filtered`, `/cmd_vel` twist bridge — runs
in AutoDRIVE with the driver's own launch:

```
autodrive-autonomy                 # TF + EKF + twist bridge
autodrive-autonomy slam:=true      # + slam_toolbox mapping
autodrive-autonomy nav:=true use_map:=<name>   # + Nav2 on a saved map
```

The driver's `racecar` shell tool is sourced too, so the car's own commands
work where they don't need hardware:

```
racecar mapping               # SLAM (slam_toolbox; gmapping/cartographer as args)
racecar mapping save <name>   # save the map
racecar navigation <name>     # Nav2 on a saved map
```

Maps save to `~/osracer_ws/src/osracer/osracer_slam/maps`, identical to the
car. `racecar service` and other hardware-bound subcommands do not apply to
AutoDRIVE — there are no hardware devices here; `autodrive` is AutoDRIVE's
teleop.

## Topic contract

Command chain, identical to the car:

```
lab (/drive) → mux_node (/mux_out) → throttle_node (/motor) → bridge → AutoDRIVE
```

The mux publishes at 50 Hz and zeroes the command when the lab goes stale
(the car's watchdog). The throttle applies the car's actuator caps. The
bridge converts `/motor` wire units to wheel units: the firmware maps wire
±30° onto the full servo swing, and the linkage reaches true 30° wheel lock
at 0.625 of it, so `/motor` 0.625 = 30° wheels = AutoDRIVE full lock.

| Topic | Type | Notes |
|---|---|---|
| `/scan` | LaserScan | 1440-bin full circle (LakiBeam wire format), 270° live window, rear wedge inf, frame `laser`, 30 Hz, RELIABLE |
| `/camera/color` | Image | JPEG bytes, encoding `jpeg`, frame `camera_link`, 60 fps |
| `/imu/fused` | Imu | frame `imu_link`, RELIABLE depth 10 |
| `/odom` | Odometry | odom → base_footprint, RELIABLE depth 10 |
| `/battery` | BatteryState | static pack voltage (AutoDRIVE has no battery model) |
| `/battery/voltage` | Float32 | racecar_neo scalar contract; `/battery/current` deliberately absent (no shunt on the car) |
| `/encoder/speed` | Float32 | motor-encoder ground speed, m/s |
| `/rc/channels` | Float32MultiArray | 10 channels, transmitter-off neutral |
| `/edgetpu/inference` | Detection2DArray | YOLO detection node (driver's own, default on; ~90 s warmup after boot) |
| `/drive` | AckermannDriveStamped | normalized lab commands into the mux |
| `/motor` | AckermannDriveStamped | throttle output, wire units |
| `/joy` | Joy | virtual gamepad, auto-START |

Topic names follow the driver's racecar_neo contract. The five lab
dashboards (camlabel, wallfollow, pursuit, eps, smartfollow) install
disabled, exactly as the car's installer leaves them; manage them with the
same commands as on the car:

```
racecar service status              # core stack + lab dashboards
racecar service start wallfollow    # start one for a lab session
racecar service stop wallfollow
```

racecar_core conventions carry over: lidar samples in centimeters, index 0
is the leftmost ray, positive steering angle means right.

## Running in a container

The services replicate the physical car's operational surface (auto-start and
`racecar service` management). Every service wraps exactly one foreground
command. Docker has no systemd init, so `autodrive_install.sh` installs supervisord
and a small compatibility backend for the existing `racecar` shell tool. The
`racecar service` commands remain unchanged. To run the components manually:

```
ros2 launch autodrive_neoracer autodrive.launch.py          # AutoDRIVE stack
ros2 launch autodrive_neoracer autonomy.launch.py           # autonomy base
python3 ~/neoracer_ros2_driver/scripts/dashboards/wallfollow_dashboard/wallfollow.py
~/.local/bin/jupyter-lab --ip=0.0.0.0 --notebook-dir=$HOME/jupyter_ws
```

Labs and demos are unaffected: they only talk to ROS topics and cannot
tell which supervisor started the graph. Expose ports 4567 (AutoDRIVE
bridge), 8080-8085 (dashboards), and 8888 (JupyterLab) from the container
as needed. Without a GPU passed through, launch the AutoDRIVE stack with
`inference:=false`; everything except the detection topic works the same.

Two container specifics: clone this repository with
`--depth 1 --single-branch` (it carries the full AutoDRIVE history; a
shallow clone avoids a multi-gigabyte download), and connect AutoDRIVE
to the Docker host's address, not the container-internal IP - the published
ports forward inward.

## Troubleshooting

- `autodrive: command not found` — open a new terminal, or `source ~/.bashrc`.
- AutoDRIVE Simulator connects but nothing moves, frame counter stuck at 1 —
  the bridge was not listening when you pressed Connect. Stop Play, start
  `autodrive`, Play and
  Connect again.
- `ros2 topic list` wrong or lab ignored — stale DDS state:
  `ros2 daemon stop`, `rm -f /dev/shm/fastrtps_*`, restart both terminals.
- `ModuleNotFoundError: rclpy` or numpy errors — a virtualenv is shadowing
  the system Python. `deactivate`; labs must run on `/usr/bin/python3`.
- `ModuleNotFoundError: nav2_msgs` — the apt packages from Part 2 are not
  installed; rerun `autodrive_install.sh` or install them directly.
- `import racecar_core` fails — the selection file is missing. Write the
  library path into
  `$(python3 -c 'import site; print(site.getusersitepackages())')/racecar_student.pth`.
- WSL2: AutoDRIVE cannot connect to `127.0.0.1` — expected; Windows forwards
  localhost to WSL over IPv6 only. Use the `hostname -I` address, and
  re-check it after a reboot.

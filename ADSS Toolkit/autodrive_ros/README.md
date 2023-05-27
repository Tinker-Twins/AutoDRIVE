# ROS Package

<p align="justify">
This directory hosts the <code>autodrive</code> ROS package, which supports modular algorithm development targetted towards autonomous driving. It uses client libraries for Python and C++, and can be therefore exploited by the users to develop their algorithms flexibly.
</p>

## SETUP

1. Clone `AutoDRIVE-Devkit` branch of the `AutoDRIVE` repository.
    ```bash
    $ git clone --single-branch --branch AutoDRIVE-Devkit https://github.com/Tinker-Twins/AutoDRIVE.git
    ```
2. Move the `autodrive` ROS package to the source space (`src`) of your catkin workspace.
    ```bash
    $ mv ~/AutoDRIVE-Devkit/autodrive_ros/autodrive ~/catkin_ws/src/
    ```
3. Build the package.
    ```bash
    $ cd ~/catkin_ws
    $ catkin_make
    ```

## USAGE

### Usage with AutoDRIVE Simulator

- **Headless Mode Bringup:**
  ```bash
  $ roslaunch autodrive simulator_bringup_headless.launch
  ```

- **RViz Mode Bringup:**
  ```bash
  $ roslaunch autodrive simulator_bringup_rviz.launch
  ```

### Usage with AutoDRIVE Testbed

- **Central Teleoperation (From Vehicle PC)**
  ```bash
  $ roslaunch autodrive testbed_teleop.launch
  ```

- **Remote Teleoperation (From Remote PC)**

  **On Vehicle PC:**
  ```bash
  $ roslaunch autodrive testbed_bringup.launch
  ```
  
  **On Remote PC:**
  ```bash
  $ roslaunch autodrive testbed_teleop_remote.launch
  ```

## TROUBLESHOOTING

Upon execution of any launch file, if you encounter an error saying "Make sure file exists in package path and permission is set to executable (chmod +x)", set the executable flag on the process for all scripts and executables by running `chmod` on the entire workspace directory (won't hurt if it's already set)

```bash
$ cd <path/to/AutoDRIVE/Workspace>
$ sudo chmod +x -R *
```

# AutoDRIVE DevKit | ROS Package

This directory hosts the `autodrive` ROS package, which supports modular algorithm development targetted towards autonomous driving. It uses client libraries for C++ and Python, and can be therefore exploited by the users to develop their algorithms flexibly; this also opens up a wide avenue for the development of hybrid high-performance algorithms aimed at autonomous driving.

## SETUP

1. Clone `AutoDRIVE-DevKit` branch of the `AutoDRIVE` repository.
```bash
$ git clone --single-branch --branch AutoDRIVE-DevKit https://github.com/Tinker-Twins/AutoDRIVE.git
```
2. Move the `autodrive` ROS package to the source space (`src`) of your catkin workspace.
```bash
$ mv ~/AutoDRIVE-DevKit/autodrive_ros/autodrive ~/catkin_ws/src/
```
3. Build the package.
```bash
$ cd ~/catkin_ws
$ catkin_make
```

## USAGE

- **Headless Mode:** Launch the `autodrive.launch` file.
```bash
$ roslaunch autodrive autodrive.launch
```

- **RViz Mode:** Launch the `autodrive_rviz.launch` file.
```bash
$ roslaunch autodrive autodrive_rviz.launch
```

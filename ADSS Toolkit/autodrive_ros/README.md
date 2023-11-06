# ROS Package

<p align="justify">
This directory hosts ROS API (a meta-package), which supports modular algorithm development targetted towards autonomous driving. It uses client libraries for Python and C++, and can be therefore exploited by the users to develop their algorithms flexibly.
</p>

## DEPENDENCIES

[AutoDRIVE Devkit's ROS API](https://github.com/Tinker-Twins/AutoDRIVE/tree/AutoDRIVE-Devkit/ADSS%20Toolkit/autodrive_ros) has the following dependencies (tested with Python 3.8.10):

- Websocket-related dependencies for communication bridge between [AutoDRIVE Simulator](https://github.com/Tinker-Twins/AutoDRIVE/tree/AutoDRIVE-Simulator) and [AutoDRIVE Devkit](https://github.com/Tinker-Twins/AutoDRIVE/tree/AutoDRIVE-Devkit) (version sensitive):
  | Package | Tested Version |
  |---------|----------------|
  | eventlet | 0.33.3 |
  | Flask | 1.1.1 |
  | Flask-SocketIO | 4.1.0 |
  | python-socketio | 4.2.0 |
  | python-engineio | 3.13.0 |
  | greenlet | 1.0.0 |
  | gevent | 21.1.2 |
  | gevent-websocket | 0.10.1 |
  
  ```bash
  $ pip3 install eventlet==0.33.3
  $ pip3 install Flask==1.1.1
  $ pip3 install Flask-SocketIO==4.1.0
  $ pip3 install python-socketio==4.2.0
  $ pip3 install python-engineio==3.13.0
  $ pip3 install greenlet==1.0.0
  $ pip3 install gevent==21.1.2
  $ pip3 install gevent-websocket==0.10.1
  ```

- Generic dependencies for data processing and visualization (usually any version will do the job):

  | Package | Tested Version |
  |---------|----------------|
  | attrdict | 2.0.1 |
  | numpy | 1.13.3 |
  | pillow | 5.1.0 |
  | opencv-contrib-python | 4.5.1.48 |

  ```bash
  $ pip3 install attrdict
  $ pip3 install numpy
  $ pip3 install pillow
  $ pip3 install opencv-contrib-python
  ```

- ROS package dependencies:

  | Package | Tested Version |
  |---------|----------------|
  | [rplidar_ros](http://wiki.ros.org/rplidar_ros) | Noetic |
  | [hector_slam](http://wiki.ros.org/hector_slam) | Noetic |
  | [map_server](http://wiki.ros.org/map_server) | Noetic |
  | [amcl](http://wiki.ros.org/amcl) | Noetic |
  | [navigation](https://wiki.ros.org/navigation) | Noetic |
  | [teb_local_planner](http://wiki.ros.org/teb_local_planner) | Noetic |

  ```bash
  $ sudo apt-get install ros-$ROS_DISTRO-rplidar-ros
  $ sudo apt-get install ros-$ROS_DISTRO-hector-slam
  $ sudo apt-get install ros-$ROS_DISTRO-map-server
  $ sudo apt-get install ros-$ROS_DISTRO-amcl
  $ sudo apt-get install ros-$ROS_DISTRO-navigation
  $ sudo apt-get install ros-$ROS_DISTRO-teb-local-planner
  ```

## SETUP

1. Clone `AutoDRIVE-Devkit` branch of the `AutoDRIVE` repository.
    ```bash
    $ git clone --single-branch --branch AutoDRIVE-Devkit https://github.com/Tinker-Twins/AutoDRIVE.git
    ```
2. Move the `autodrive_ros` ROS meta-package to the source space (`src`) of your catkin workspace.
    ```bash
    $ mv ~/AutoDRIVE-Devkit/autodrive_ros ~/catkin_ws/src/
    ```
3. Build the package.
    ```bash
    $ cd ~/catkin_ws
    $ catkin_make
    ```

## USAGE

### Usage with Nigel

#### Usage with AutoDRIVE Simulator

- **Bringup:**
  - **Headless Mode Bringup:**
    ```bash
    $ roslaunch autodrive_nigel simulator_bringup_headless.launch
    ```
    **[OR]**
  - **RViz Mode Bringup:**
    ```bash
    $ roslaunch autodrive_nigel simulator_bringup_rviz.launch
    ```
  
- **Teleoperation:**
  ```bash
  $ roslaunch autodrive_nigel simulator_teleop.launch
  ```
  
- **Odometry:**
  ```bash
  $ roslaunch autodrive_nigel simulator_lidar_odometry.launch
  ```

- **Simultaneous Localization & Mapping (SLAM):**
  ```bash
  # Map the environment
  $ roslaunch autodrive_nigel simulator_hector_slam.launch
  # Save the map
  $ rosrun map_server map_saver -f my_map
  ``` 
  
- **Map-Based Localization:**
  ```bash
  $ roslaunch autodrive_nigel simulator_amcl.launch
  ``` 

- **Autonomous Navigation:**
  ```bash
  # Begin autonomous navigation
  $ roslaunch autodrive_nigel simulator_navigation.launch
  # Publish navigation goal from script
  $ roslaunch autodrive_nigel simulator_navigation_goal.launch
  ```

#### Usage with AutoDRIVE Testbed

- **Central Teleoperation (From Vehicle PC)**
  ```bash
  $ roslaunch autodrive_nigel testbed_teleop.launch
  ```

- **Central Odometry (From Vehicle PC)**
  ```bash
  $ roslaunch autodrive_nigel testbed_lidar_odometry.launch
  ```

- **Central Simultaneous Localization & Mapping (SLAM) (From Vehicle PC)**
  ```bash
  # Map the environment
  $ roslaunch autodrive_nigel testbed_hector_slam.launch
  # Save the map
  $ rosrun map_server map_saver -f my_map
  ```
  
- **Central Map-Based Localization (From Vehicle PC)**
  ```bash
  $ roslaunch autodrive_nigel testbed_amcl.launch
  ```

- **Central Autonomous Navigation (From Vehicle PC)**
  ```bash
  # Begin autonomous navigation
  $ roslaunch autodrive_nigel testbed_navigation.launch
  # Publish navigation goal from script
  $ roslaunch autodrive_nigel testbed_navigation_goal.launch
  ```

- **Remote Teleoperation (From Remote PC)**

  **On Vehicle PC:**
  ```bash
  $ roslaunch autodrive_nigel testbed_bringup.launch
  ```
  
  **On Remote PC:**
  ```bash
  $ roslaunch autodrive_nigel testbed_teleop_remote.launch
  ```

- **Remote Odometry (From Remote PC)**

  **On Vehicle PC:**
  ```bash
  $ roslaunch autodrive_nigel testbed_bringup.launch
  ```
  
  **On Remote PC:**
  ```bash
  $ roslaunch autodrive_nigel testbed_lidar_odometry_remote.launch
  ```
  
- **Remote Simultaneous Localization & Mapping (SLAM) (From Remote PC)**

  **On Vehicle PC:**
  ```bash
  $ roslaunch autodrive_nigel testbed_bringup.launch
  ```
  
  **On Remote PC:**
  ```bash
  # Map the environment
  $ roslaunch autodrive_nigel testbed_hector_slam_remote.launch
  # Save the map
  $ rosrun map_server map_saver -f my_map
  ```

- **Remote Map-Based Localization (From Remote PC)**

  **On Vehicle PC:**
  ```bash
  $ roslaunch autodrive_nigel testbed_bringup.launch
  ```
  
  **On Remote PC:**
  ```bash
  $ roslaunch autodrive_nigel testbed_amcl_remote.launch
  ```

- **Remote Autonomous Navigation (From Remote PC)**

  **On Vehicle PC:**
  ```bash
  $ roslaunch autodrive_nigel testbed_bringup.launch
  ```
  
  **On Remote PC:**
  ```bash
  # Begin autonomous navigation
  $ roslaunch autodrive_nigel testbed_navigation_remote.launch
  # Publish navigation goal from script
  $ roslaunch autodrive_nigel testbed_navigation_goal.launch
  ```

### Usage with F1TENTH

  - **Bringup:**
    - **Headless Mode Bringup:**
      ```bash
      $ roslaunch autodrive_f1tenth simulator_bringup_headless.launch
      ```
      **[OR]**
    - **RViz Mode Bringup:**
      ```bash
      $ roslaunch autodrive_f1tenth simulator_bringup_rviz.launch
    ```
  
- **Teleoperation:**
  ```bash
  $ roslaunch autodrive_f1tenth simulator_teleop.launch
  ```

## TROUBLESHOOTING

Upon execution of any launch file, if you encounter an error saying "Make sure file exists in package path and permission is set to executable (chmod +x)", set the executable flag on the process for all scripts and executables by running `chmod` on the entire workspace directory (won't hurt if it's already set)

```bash
$ cd <path/to/AutoDRIVE/Workspace>
$ sudo chmod +x -R *
```

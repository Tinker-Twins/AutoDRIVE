<p align="center">
  <img src="https://github.com/Tinker-Twins/AutoDRIVE/blob/AutoDRIVE/Images/AutoDRIVE%20Logo.png" width="256" height="256"><br>
  <b>An Integrated Platform for Autonomous Driving Research and Education</b>
</p>

#### [AutoDRIVE Testbed](https://github.com/Tinker-Twins/AutoDRIVE/tree/AutoDRIVE-Testbed)
<p align="justify">
AutoDRIVE Testbed is the hardware setup comprising of a scaled vehicle model (called Nigel) and a modular infrastructure development kit. The vehicle is equipped with a comprehensive sensor suite for redundant perception, a set of actuators for constrained motion control and a fully functional lighting system for illumination and signalling. It can be teleoperated (in manual mode) or self-driven (in autonomous mode). The infrastructure development kit comprises of various environment modules along with active and passive traffic elements.
</p>

#### [AutoDRIVE Simulator](https://github.com/Tinker-Twins/AutoDRIVE/tree/AutoDRIVE-Simulator)
<p align="justify">
AutoDRIVE Simulator is the digital twin of the AutoDRIVE Testbed, which enables the users to virtually prototype their algorithms either due to hardware limitations or as a part of the reiterative development cycle. It is developed atop the Unity game engine and offers a WebSocket interface for bilateral communication with the autonomy algorithms developed independently using the AutoDRIVE DevKit. The standalone simulator application is targeted at Full HD resolution (1920x1080p) with cross-platform support (Windows, macOS and Linux). It is light-weight (~110 MB) and utilizes system resources wisely, which enables deployment of the simulator application and autonomy algorithms on a single machine; nonetheless, distributed computing is also supported.
</p>

#### [AutoDRIVE DevKit](https://github.com/Tinker-Twins/AutoDRIVE/tree/AutoDRIVE-DevKit)
<p align="justify">
AutoDRIVE DevKit is a developer's kit that enables the users to exploit AutoDRIVE Simulator or AutoDRIVE Testbed for rapid and flexible development of autonomy algorithms. It supports local (decentralized) as well as distributed (centralized) computing and is compatible with Robot Operating System (ROS), while also offering a direct scripting support for Python and C++.
</p>
  
### Development
- [ ] AutoDRIVE Testbed
  - [ ] Vehicle (Nigel)
    - [x] Component Finalization
    - [x] Part Modelling
    - [x] CAD Assembly
    - [ ] [Optional] Wiring in CAD Assembly
    - [ ] Rendering
    - [x] Mechanical System Testing
    - [x] Chassis Fabrication
    - [x] Electrical System Layout
    - [x] Electrical System Testing
    - [x] Auxiliary PCB Design
    - [x] Auxiliary PCB Fabrication
    - [x] System Integration
    - [x] Subsystem Testing
    - [x] Finishing Touches
    - [ ] Vehicle Teleoperation (Keyboard, Joystick)
    - [ ] Software Development
  - [ ] Infrastructure
    - [x] Map Design Finalization (Driving School)
    - [x] Map Printing (Flex)
    - [x] Map (Flex) Trimming
    - [x] Obstacle Layout Finalization
    - [ ] Traffic Sign Catalog Finalization
    - [ ] Traffic Sign Stand Design
    - [ ] Trafic Light Design
    - [x] IPS Camera Fixture Installation
    - [ ] IPS Camera Calibration
    - [x] IPS AprilTag Library Integration
    - [ ] IPS AprilTag Based Localization
- [ ] AutoDRIVE Simulator
  - [ ] Port to Unity HDRP Project
  - [ ] Update Scene Lighting
  - [ ] Update Vehcile
  - [ ] Update Environment Modules
  - [ ] Add Traffic Elements (Traffic Lights, Traffic Signs, etc.)
  - [ ] Add Driving School Map
  - [ ] Battery Status Indicator for Vehicle
  - [ ] [Optional] Settings (Simulation Settings, Vehicle Settings, Infrastructure Settings)
  - [ ] [Optional] Controller Settings (Keyboard, Mouse, Joystick, etc.)
  - [ ] [Optional] HIL Simulation
  - [ ] [Optional] AR/VR
  - [ ] [Optional] Stand Alone Scene Reconfiguration
  - [ ] [Optional] Multi-Vehicle Support
  - [ ] [Optional] V2X Implementation
  - [ ] [Optional] Visualization Overlay (Sensors, Planners, Dynamics, etc.)
  - [ ] [Optional] Ground Truth Data Streaming
  
### Demonstrations
- [ ] Modular Approach
  - [ ] Perception Module
    - Lane Detection & Tracking (Basic & Advanced)
    - Object Detection & Tracking
    - Traffic Light Detection
    - Traffic Sign Detection and Classification (Sim2Real)
    - Semantic Segmentation
    - SLAM (HectorSLAM, Cartographer, ORB-SLAM)
    - Odometry (Wheel Encoder, Visual)
    - Dead Reckoning (Wheel Encoder + IMU)
    - Localization (AMCL, AptilTag)
  - [ ] Planning Module
    - Global Planning (Hybrid A*)
    - Local Planning (FSM, DWA, etc.)
  - [ ] Control Module
    - PID Control
    - Pure Pursuit Control
    - Stanley Control
    - Model Predictive Control
    - Novel Control Algorithm
- [ ] End-to-End Approach
  - [ ] Imitation Learning
    - Behavioral Cloning (Sim2Real)
  - [ ] Reinfrocement Learning
    - On-Road Navigation (LKA Scenario)
    - Intersection Management (Intersection Traversal Scenario)

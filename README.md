# AutoDRIVE
### An Integrated Platform for Autonomous Vehicle Research and Education

### Project Outline:
- AutoDRIVE Testbed
  - Vehicle
  - Environment
- AutoDRIVE Simulator
  - Digital Twin of AutoDRIVE Testbed
  - WebSocket Interface
- AutoDRIVE DevKit
  - ROS Package
  - Python API
  - C++ API
  
### Development
- [ ] AutoDRIVE Testbed
  - [ ] Vehicle (MAVE)
    - [x] Component Finalization
    - [x] Part Modelling
    - [x] CAD Assembly
    - [x] Mechanical System Testing
    - [x] Chassis Fabrication
    - [x] Electrical System Layout
    - [x] Electrical System Testing
    - [x] Auxiliary PCB Design
    - [x] Auxiliary PCB Fabrication
    - [ ] System Integration
  - [ ] Environment
    - [x] Map Design Finalization (Driving School)
    - [x] Map Fabrication
    - [ ] Obstacle Layout Finalization
    - [ ] Traffic Sign Design
    - [ ] Trafic Light Design
    - [x] IPS Camera Fixture Installation
    - [ ] IPS AprilTag Localization
- [ ] AutoDRIVE Simulator
  - [ ] Update Vehcile
  - [ ] Update Scene Lighting
  - [ ] Add Testbed Environment Scene
  - [ ] Single and Dual Lane Road Kits
  - [ ] Battery Status Indicator for Vehicle
  - [ ] Settings (Simulation Settings, Vehicle Settings, Environment Settings)
  - [ ] Active and Passive Environment Elements (Traffic Signs, Signals, etc.)
  - [ ] Controller Settings (Keyboard, Mouse, Joystick, etc.)
  - [ ] [Optional] HIL Simulation
  - [ ] [Optional] Stand Alone Scene Reconfiguration
  - [ ] [Optional] Multi-Vehicle Support
  - [ ] [Optional] V2X Implementation
  - [ ] [Optional] Visualization Overlay (Sensors, Planners, Dynamics, etc.)
  
### Demonstrations
- [ ] Modular Approach
  - [ ] Perception Module
    - Lane Detection & Tracking (Basic & Advanced)
    - Object Detection & Tracking
    - Traffic Light Detection
    - Traffic Sign Detection and Classification (Sim2Real)
    - Semantic Segmentation
    - SLAM (HectorSLAM, Cartographer, ORBSLAM, etc.)
    - Odometry (Wheel Encoder, Visual)
    - Localization (AMCL, AptilTag)
  - [ ] Planning Module
    - Global Planning (Hybrid A*)
    - Local Planning (FSM, Dynamic Window Approach, etc.)
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

<p align="center">
  <img src="https://github.com/Tinker-Twins/AutoDRIVE/blob/AutoDRIVE/Images/AutoDRIVE%20Logo.png" width="256" height="256"><br>
  <b>An Integrated Platform for Autonomous Driving Research and Education</b>
</p>

## Project Overview

![](https://github.com/Tinker-Twins/AutoDRIVE/blob/AutoDRIVE/Images/AutoDRIVE%20Overview.png)

<p align="justify">
AutoDRIVE is envisioned to be an integrated platform for autonomous driving research and education. It bridges the gap between software simulation and hardware deployment by providing the AutoDRIVE Simulator and AutoDRIVE Testbed, a well-suited duo for sim2real applications. It also offers AutoDRIVE DevKit, a developer's kit for rapid and flexible development of autonomy algorithms. Although the platform is primarily targeted towards autonomous driving, it also supports the development of smart-city solutions for managing the traffic flow.
</p>

## AutoDRIVE Testbed

| <img src="https://github.com/Tinker-Twins/AutoDRIVE/blob/AutoDRIVE/Images/Testbed%20Vehicle.png" width="500"> | <img src="https://github.com/Tinker-Twins/AutoDRIVE/blob/AutoDRIVE/Images/Testbed%20Infrastructure.png" width="600"> |
|:--------:|:-------------:|
| Vehicle | Infrastructure |

<p align="justify">
AutoDRIVE Testbed is the hardware setup comprising of a scaled vehicle model (called Nigel) and a modular infrastructure development kit. The vehicle is equipped with a comprehensive sensor suite for redundant perception, a set of actuators for constrained motion control and a fully functional lighting system for illumination and signaling. It can be teleoperated (in manual mode) or self-driven (in autonomous mode). The infrastructure development kit comprises of various environment modules along with active and passive traffic elements.
</p>

- **Source Branch:** [AutoDRIVE Testbed](https://github.com/Tinker-Twins/AutoDRIVE/tree/AutoDRIVE-Testbed)
- **Latest Release:** None
- **Upcoming Release:** AutoDRIVE Testbed 0.1.0 is currently under development.

## AutoDRIVE Simulator
<p align="justify">
AutoDRIVE Simulator is the digital twin of the AutoDRIVE Testbed, which enables the users to virtually prototype their algorithms either due to hardware limitations or as a part of the reiterative development cycle. It is developed atop the Unity game engine and offers a WebSocket interface for bilateral communication with the autonomy algorithms developed independently using the AutoDRIVE DevKit. The standalone simulator application is targeted at Full HD resolution (1920x1080p) with cross-platform support (Windows, macOS and Linux). It is a light-weight software application that utilizes system resources wisely. This enables deployment of the simulator application and autonomy algorithms on a single machine; nonetheless, distributed computing is also supported.
</p>

- **Source Branch:** [AutoDRIVE Simulator](https://github.com/Tinker-Twins/AutoDRIVE/tree/AutoDRIVE-Simulator)
- **Latest Release:** [AutoDRIVE Simulator 0.1.1](https://github.com/Tinker-Twins/AutoDRIVE/releases/tag/Simulator-0.1.1)
- **Upcoming Release:** AutoDRIVE Simulator 0.2.0 is currently under development.

## AutoDRIVE DevKit
<p align="justify">
AutoDRIVE DevKit is a developer's kit that enables the users to exploit AutoDRIVE Simulator or AutoDRIVE Testbed for rapid and flexible development of autonomy algorithms. It supports local (decentralized) as well as distributed (centralized) computing and is compatible with Robot Operating System (ROS), while also offering a direct scripting support for Python and C++.
</p>

- **Source Branch:** [AutoDRIVE DevKit](https://github.com/Tinker-Twins/AutoDRIVE/tree/AutoDRIVE-DevKit)
- **Latest Release:** [AutoDRIVE DevKit 0.1.0](https://github.com/Tinker-Twins/AutoDRIVE/releases/tag/DevKit-0.1.0)
- **Upcoming Release:** AutoDRIVE DevKit 0.2.0 is currently under development.

## Demonstrations

Implementation demonstrations are available on [YouTube](https://youtube.com/playlist?list=PLY45pkzWzH9_iRlOqmqvFdQPASwevTtbW).

|                    |                     |
|:------------------:|:-------------------:|
| [<img src="https://github.com/Tinker-Twins/AutoDRIVE/blob/AutoDRIVE/Images/Autonomous%20Parking.png" width="500">](https://youtu.be/oBqIZZA0wkc) | [<img src="https://github.com/Tinker-Twins/AutoDRIVE/blob/AutoDRIVE/Images/Behavioural%20Cloning.png" width="500">](https://youtu.be/rejpoogaXOE) |
| [Autonomous Parking](https://youtu.be/oBqIZZA0wkc) | [Behavioural Cloning](https://youtu.be/rejpoogaXOE) |
| [<img src="https://github.com/Tinker-Twins/AutoDRIVE/blob/AutoDRIVE/Images/Intersection%20Traversal.png" width="500">](https://youtu.be/AEFJbDzOpcM) | [<img src="https://github.com/Tinker-Twins/AutoDRIVE/blob/AutoDRIVE/Images/Smart%20City%20Management.png" width="500">](https://youtu.be/fnxOpV1gFXo) |
| [Intersection Traversal](https://youtu.be/AEFJbDzOpcM) | [Smart City Management](https://youtu.be/fnxOpV1gFXo) |
|                    |                     |

## Citation

We encourage you to cite the following papers if you use any part of this project for your research:

#### [AutoDRIVE Simulator: A Simulator for Scaled Autonomous Vehicle Research and Education](https://arxiv.org/abs/2103.10030)
```bibtex
@inproceedings{AutoDRIVE-Simulator-2021,
author = {Samak, Tanmay Vilas and Samak, Chinmay Vilas and Xie, Ming},
title = {AutoDRIVE Simulator: A Simulator for Scaled Autonomous Vehicle Research and Education},
year = {2021},
isbn = {9781450390453},
publisher = {Association for Computing Machinery},
address = {New York, NY, USA},
url = {https://doi.org/10.1145/3483845.3483846},
doi = {10.1145/3483845.3483846},
booktitle = {2021 2nd International Conference on Control, Robotics and Intelligent System},
pages = {1–5},
numpages = {5},
location = {Qingdao, China},
series = {CCRIS'21}
}
```
This work has been published in **2021 International Conference on Control, Robotics and Intelligent System (CCRIS).** The publication can be found on [ACM Digital Library](https://dl.acm.org/doi/abs/10.1145/3483845.3483846).

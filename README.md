<p align="center">
  <img src="https://github.com/Tinker-Twins/AutoDRIVE/blob/AutoDRIVE/Images/AutoDRIVE%20Logo.png" width="256" height="256"><br>
  <b>An Integrated Platform for Autonomous Driving Research and Education</b>
</p>

## Project Overview

![](https://github.com/Tinker-Twins/AutoDRIVE/blob/AutoDRIVE/Images/AutoDRIVE%20Overview.png)

<p align="justify">
AutoDRIVE is envisioned to be an integrated platform for autonomous driving research and education. It bridges the gap between software simulation and hardware deployment by providing the AutoDRIVE Simulator and AutoDRIVE Testbed, a well-suited duo for sim2real applications. It also offers AutoDRIVE Devkit, a developer's kit for rapid and flexible development of autonomy algorithms. Although the platform is primarily targeted towards autonomous driving, it also supports the development of smart-city solutions for managing the traffic flow.
</p>

## AutoDRIVE Testbed

| <img src="https://github.com/Tinker-Twins/AutoDRIVE/blob/AutoDRIVE/Images/Testbed%20Vehicle.png" width="500"> | <img src="https://github.com/Tinker-Twins/AutoDRIVE/blob/AutoDRIVE/Images/Testbed%20Infrastructure.png" width="500"> |
|:--------:|:-------------:|
| Vehicle | Infrastructure |

<p align="justify">
AutoDRIVE Testbed is the hardware setup comprising of a scaled vehicle model (called Nigel) and a modular infrastructure development kit. The vehicle is equipped with a comprehensive sensor suite for redundant perception, a set of actuators for constrained motion control and a fully functional lighting system for illumination and signaling. It can be teleoperated (in manual mode) or self-driven (in autonomous mode). The infrastructure development kit comprises of various environment modules along with active and passive traffic elements.
</p>

- **Source Branch:** [AutoDRIVE Testbed](https://github.com/Tinker-Twins/AutoDRIVE/tree/AutoDRIVE-Testbed)
- **Latest Release:** None
- **Upcoming Release:** AutoDRIVE Testbed 0.1.0 is currently under development.

## AutoDRIVE Simulator

| <img src="https://github.com/Tinker-Twins/AutoDRIVE/blob/AutoDRIVE/Images/Simulator%20Vehicle.png" width="500"> | <img src="https://github.com/Tinker-Twins/AutoDRIVE/blob/AutoDRIVE/Images/Simulator%20Infrastructure.png" width="500"> |
|:--------:|:-------------:|
| Vehicle | Infrastructure |

<p align="justify">
AutoDRIVE Simulator is the digital twin of the AutoDRIVE Testbed, which enables the users to virtually prototype their algorithms either due to hardware limitations or as a part of the reiterative development cycle. It is developed atop the Unity game engine and offers a WebSocket interface for bilateral communication with the autonomy algorithms developed independently using the AutoDRIVE Devkit. The standalone simulator application is targeted at Full HD resolution (1920x1080p) with cross-platform support (Windows, macOS and Linux). It is a light-weight software application that utilizes system resources wisely. This enables deployment of the simulator application and autonomy algorithms on a single machine; nonetheless, distributed computing is also supported.
</p>

- **Source Branch:** [AutoDRIVE Simulator](https://github.com/Tinker-Twins/AutoDRIVE/tree/AutoDRIVE-Simulator)
- **Latest Release:** [AutoDRIVE Simulator 0.1.1](https://github.com/Tinker-Twins/AutoDRIVE/releases/tag/Simulator-0.1.1)
- **Upcoming Release:** AutoDRIVE Simulator 0.2.0 is currently under development.

## AutoDRIVE Devkit

| <img src="https://github.com/Tinker-Twins/AutoDRIVE/blob/AutoDRIVE/Images/ADSS.png" width="500"> | <img src="https://github.com/Tinker-Twins/AutoDRIVE/blob/AutoDRIVE/Images/SCSS.png" width="500"> |
|:--------:|:-------------:|
| ADSS Toolkit | SCSS Toolkit |

<p align="justify">
AutoDRIVE Devkit is a developer's kit that enables the users to exploit AutoDRIVE Simulator or AutoDRIVE Testbed for rapid and flexible development of autonomy algorithms pertaining to autonomous driving (using ADSS Toolkit) as well as smart city management (using SCSS Toolkit). It supports local (decentralized) as well as distributed (centralized) computing and is compatible with Robot Operating System (ROS), while also offering a direct scripting support for Python and C++.
</p>

- **Source Branch:** [AutoDRIVE Devkit](https://github.com/Tinker-Twins/AutoDRIVE/tree/AutoDRIVE-Devkit)
- **Latest Release:** [AutoDRIVE Devkit 0.1.0](https://github.com/Tinker-Twins/AutoDRIVE/releases/tag/Devkit-0.1.0)
- **Upcoming Release:** AutoDRIVE Devkit 0.2.0 is currently under development.

## Resources

### Presentations

We encourage you to take a look at the following presentations to gain a better insight into the AutoDRIVE project.

|                    |                     |
|:------------------:|:-------------------:|
| [<img src="https://github.com/Tinker-Twins/AutoDRIVE/blob/AutoDRIVE/Images/AutoDRIVE%20Simulator%20Pitch%20Video.png" width="500">](https://youtu.be/i7R79jwnqlg) | [<img src="https://github.com/Tinker-Twins/AutoDRIVE/blob/AutoDRIVE/Images/AutoDRIVE%20Testbed%20Pitch%20Video.png" width="500">](https://youtu.be/YFQzyfXV6Rw) |
| [AutoDRIVE Simulator Pitch Video](https://youtu.be/i7R79jwnqlg) | [AutoDRIVE Testbed Pitch Video](https://youtu.be/YFQzyfXV6Rw) |
| [<img src="https://github.com/Tinker-Twins/AutoDRIVE/blob/AutoDRIVE/Images/SRMIST%20FYP%20Viva%20Voce.png" width="500">](https://youtu.be/2FByDOkDxMc) | [<img src="https://github.com/Tinker-Twins/AutoDRIVE/blob/AutoDRIVE/Images/CCRIS%202021%20Presentation.png" width="500">](https://youtu.be/whTH6VyVtHE) |
| [SRMIST UG Final Year Project Viva Voce](https://youtu.be/2FByDOkDxMc) | [CCRIS 2021 Virtual Presentation](https://youtu.be/whTH6VyVtHE) |
|                    |                     |

### Demonstrations

We encourage you to take a look at the following research projects developed using the AutoDRIVE platform.

|                    |                     |
|:------------------:|:-------------------:|
| [<img src="https://github.com/Tinker-Twins/AutoDRIVE/blob/AutoDRIVE/Images/Autonomous%20Parking.png" width="500">](https://youtu.be/oBqIZZA0wkc) | [<img src="https://github.com/Tinker-Twins/AutoDRIVE/blob/AutoDRIVE/Images/Behavioural%20Cloning.png" width="500">](https://youtu.be/rejpoogaXOE) |
| [Autonomous Parking](https://youtu.be/oBqIZZA0wkc) | [Behavioural Cloning](https://youtu.be/rejpoogaXOE) |
| [<img src="https://github.com/Tinker-Twins/AutoDRIVE/blob/AutoDRIVE/Images/Intersection%20Traversal.png" width="500">](https://youtu.be/AEFJbDzOpcM) | [<img src="https://github.com/Tinker-Twins/AutoDRIVE/blob/AutoDRIVE/Images/Smart%20City%20Management.png" width="500">](https://youtu.be/fnxOpV1gFXo) |
| [Intersection Traversal](https://youtu.be/AEFJbDzOpcM) | [Smart City Management](https://youtu.be/fnxOpV1gFXo) |
|                    |                     |

### Publications

We encourage you to read and cite the following papers if you use any part of this project for your research:

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

<p align="center">
  <img src="https://github.com/Tinker-Twins/AutoDRIVE/blob/AutoDRIVE/Images/AutoDRIVE%20Logo.png" width="256" height="256"><br>
  <b>An Integrated Platform for Autonomous Driving Research and Education</b>
</p>

## Project Overview

<p align="justify">
AutoDRIVE is envisioned to be an integrated platform for autonomous driving research and education. It bridges the gap between software simulation and hardware deployment by providing the AutoDRIVE Simulator and AutoDRIVE Testbed, a well-suited duo for sim2real applications. It also offers AutoDRIVE DevKit, a developer's kit for rapid and flexible development of autonomy algorithms. Although the platform is primarily targeted towards autonomous driving, it also supports the development of smart-city solutions for managing the traffic flow.
</p>

## AutoDRIVE Testbed
<p align="justify">
AutoDRIVE Testbed is the hardware setup comprising of a scaled vehicle model (called Nigel) and a modular infrastructure development kit. The vehicle is equipped with a comprehensive sensor suite for redundant perception, a set of actuators for constrained motion control and a fully functional lighting system for illumination and signalling. It can be teleoperated (in manual mode) or self-driven (in autonomous mode). The infrastructure development kit comprises of various environment modules along with active and passive traffic elements.
</p>

- **Latest Release:** None
- **Upcoming Release:** AutoDRIVE Testbed v0.1.0 is currently under development.
- **Source Branch:** [AutoDRIVE Testbed](https://github.com/Tinker-Twins/AutoDRIVE/tree/AutoDRIVE-Testbed)

## AutoDRIVE Simulator
<p align="justify">
<a href="https://github.com/Tinker-Twins/AutoDRIVE/tree/AutoDRIVE-Simulator">AutoDRIVE Simulator</a> is the digital twin of the AutoDRIVE Testbed, which enables the users to virtually prototype their algorithms either due to hardware limitations or as a part of the reiterative development cycle. It is developed atop the Unity game engine and offers a WebSocket interface for bilateral communication with the autonomy algorithms developed independently using the AutoDRIVE DevKit. The standalone simulator application is targeted at Full HD resolution (1920x1080p) with cross-platform support (Windows, macOS and Linux). It is a light-weight software application that utilizes system resources wisely. This enables deployment of the simulator application and autonomy algorithms on a single machine; nonetheless, distributed computing is also supported.
</p>

- **Latest Release:** [AutoDRIVE Simulator v0.1.1](https://github.com/Tinker-Twins/AutoDRIVE/releases/tag/Simulator-0.1.1)
- **Upcoming Release:** AutoDRIVE Simulator v0.2.0 is currently under development.

## AutoDRIVE DevKit
<p align="justify">
<a href="https://github.com/Tinker-Twins/AutoDRIVE/tree/AutoDRIVE-DevKit">AutoDRIVE DevKit</a> is a developer's kit that enables the users to exploit AutoDRIVE Simulator or AutoDRIVE Testbed for rapid and flexible development of autonomy algorithms. It supports local (decentralized) as well as distributed (centralized) computing and is compatible with Robot Operating System (ROS), while also offering a direct scripting support for Python and C++.
</p>

- **Latest Release:** [AutoDRIVE DevKit v0.1.0](https://github.com/Tinker-Twins/AutoDRIVE/releases/tag/DevKit-0.1.0)
- **Upcoming Release:** AutoDRIVE DevKit v0.2.0 is currently under development.

## Demonstrations

Implementation demonstrations are available on [YouTube](https://youtube.com/playlist?list=PLY45pkzWzH9_iRlOqmqvFdQPASwevTtbW).

## Citation

We encourage you to cite the following paper if you use any part of this project for your research:

```bibtex
@eprint{AutoDRIVE-Simulator-2021,
      title={AutoDRIVE Simulator: A Simulator for Scaled Autonomous Vehicle Research and Education}, 
      author={Tanmay Vilas Samak and Chinmay Vilas Samak and Ming Xie},
      year={2021},
      eprint={2103.10030},
      archivePrefix={arXiv},
      primaryClass={cs.RO}
}
```

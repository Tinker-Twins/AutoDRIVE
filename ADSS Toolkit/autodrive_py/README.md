# Python API

<p align="justify">
This directory hosts the Python API <code>autodrive.py</code> for AutoDRIVE, which can be used to easily develop autonomous driving algorithms, especially those employing learning-based techniques.
</p>

## DEPENDENCIES

[AutoDRIVE Devkit's Python API](https://github.com/Tinker-Twins/AutoDRIVE/tree/AutoDRIVE-Devkit/ADSS%20Toolkit/autodrive_py) has the following dependencies (tested with Python 3.6.5):

- Websocket-related dependencies for communication bridge between [AutoDRIVE Simulator 0.2.0](https://github.com/Tinker-Twins/AutoDRIVE/releases/tag/Simulator-0.2.0) and [AutoDRIVE Devkit 0.2.0](https://github.com/Tinker-Twins/AutoDRIVE/releases/tag/Devkit-0.2.0):

  | Package | Tested Version |
  |---------|----------------|
  | eventlet | 0.33.3 |
  | Flask | 1.1.1 |
  | Flask-SocketIO | 4.1.0 |
  | python-engineio | 3.13.0 |
  | python-socketio | 4.2.0 |

- Generic dependencies for data processing and visualization:

  | Package | Tested Version |
  |---------|----------------|
  | numpy | 1.13.3 |
  | pillow | 5.1.0 |
  | opencv-contrib-python | 4.5.1.48 |


## USAGE

Execute an example Python3 script employing the AutoDRIVE Python API.
```bash
$ cd ~/autodrive_py
$ python3 example.py
```

# Python API

<p align="justify">
This directory hosts the Python API <code>autodrive.py</code> for AutoDRIVE, which can be used to easily develop autonomous driving algorithms, especially those employing learning-based techniques.
</p>

## DEPENDENCIES

[AutoDRIVE Devkit's Python API](https://github.com/Tinker-Twins/AutoDRIVE/tree/AutoDRIVE-Devkit/ADSS%20Toolkit/autodrive_py) has the following dependencies (tested with Python 3.6.5):

- Websocket-related dependencies for communication bridge between [AutoDRIVE Simulator 0.2.0](https://github.com/Tinker-Twins/AutoDRIVE/releases/tag/Simulator-0.2.0) and [AutoDRIVE Devkit 0.2.0](https://github.com/Tinker-Twins/AutoDRIVE/releases/tag/Devkit-0.2.0) (version sensitive):

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
  | numpy | 1.13.3 |
  | pillow | 5.1.0 |
  | opencv-contrib-python | 4.5.1.48 |
  
  ```bash
  $ pip3 install numpy
  $ pip3 install pillow
  $ pip3 install opencv-contrib-python
  ```

## USAGE

Execute an example Python3 script employing the AutoDRIVE Python API.
```bash
$ cd ~/autodrive_py
$ python3 example.py
```

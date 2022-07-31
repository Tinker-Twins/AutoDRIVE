# C++ API

<p align="justify">
This directory hosts the C++ API for AutoDRIVE, which can be used to develop high performance autonomous driving algorithms.
</p>

## SETUP

1. Clone `AutoDRIVE-Devkit` branch of the `AutoDRIVE` repository.
    ```bash
    $ git clone --single-branch --branch AutoDRIVE-Devkit https://github.com/Tinker-Twins/AutoDRIVE.git
    ```
2. Build the source code using the `build.sh` shell script (requires [CMake](https://cmake.org/)).
    ```bash
    $ cd ~/autodrive_cpp
    $ ./build.sh
    ```
  
    _**Note:** To clean and rebuild the entire source code, use the the `clean.sh` and `build.sh` shell scripts._
    ```bash
    $ cd ~/autodrive_cpp
    $ ./clean.sh
    $ ./build.sh
    ```
## USAGE

Execute the compiled API using the `run.sh` shell script.
```bash
$ cd ~/autodrive_cpp
$ ./run.sh
```

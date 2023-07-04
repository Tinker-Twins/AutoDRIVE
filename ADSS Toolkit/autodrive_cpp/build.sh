#!/bin/bash
# Script to build (compile) the project.
# Clean existing build using "clean.sh".

# Go into the directory where this bash script is contained.
cd `dirname $0`

# Install required dependencies
sudo apt-get update
sudo apt-get install -y git libuv1-dev libssl-dev libz-dev gcc g++ cmake make
git clone https://github.com/uWebSockets/uWebSockets
cd uWebSockets
git checkout e94b6e1

# Build (compile) the project and store output in the dedicated `build` directory.
mkdir -p build
cd build
cmake ..
make
sudo make install
cd ../..
sudo ln -sf /usr/lib64/libuWS.so /usr/lib/libuWS.so

mkdir -p build
cd build
cmake .. && make
cd ..

# Clean the repository
sudo rm -r uWebSockets
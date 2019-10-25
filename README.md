# RDS
The Restrained Dynamical System (RDS) is a control method that avoids collisions for a wheeled vehicle. This repository contains (or will contain) the method's implementation, tools that allow to visualize and monitor its operation, a ROS interface, and example code and instructions to integrate, try out and demonstrate the functionality of the full system or its individual components.

## Visualization Setup

The visualization depends on the game library SDL 2.0. To install it and set it up (on Ubuntu), use the following commands in a terminal.
```
hg clone https://hg.libsdl.org/SDL SDL
cd SDL
mkdir build
cd build
../configure
make -j10
sudo make install
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib
``` 

## Demo
The demo uses the visualization and therefore requires to setup SDL as described above. To build and run the demo, run the following commands in a terminal.
```
cd path/to/rds
make demo_geo
./build/demo_geo
```

## Unit Test Setup

The implemented unit tests for individual library functions use the gtest library. The following steps install it on Ubuntu 16.04.
```
# Prerequisite: CMake
dpkg -s libgtest-dev
# if the result says it is not installed, install with: sudo apt-get install libgtest-dev
dpkg -L libgtest-dev
# this list includes for me: /usr/src/gtest 
# I heard that on newer Ubuntu versions the equivalent is: /usr/src/googletest/googletest
cd /usr/src/gtest #if this is among the listed directories, otherwise go to equivalent
sudo mkdir build
cd build
sudo cmake ..
sudo make
sudo cp libgtest* /usr/lib/
cd ..
sudo rm -rf build
sudo mkdir /usr/local/lib/gtest
sudo ln -s /usr/lib/libgtest.a /usr/local/lib/gtest/libgtest.a
sudo ln -s /usr/lib/libgtest_main.a /usr/local/lib/gtest/libgtest_main.a
```
To build and execute one of the unit tests, run for example the following commands.
```
make test_geometry
./build/test_geometry
```

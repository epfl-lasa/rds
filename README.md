# RDS
The Restrained Dynamical System (RDS) is a control method that avoids collisions for a wheeled vehicle. This repository contains (or will contain) the method's implementation, tools that allow to visualize and monitor its operation, a ROS interface, and example code and instructions to integrate, try out and demonstrate the functionality of the full system or its individual components.

## Setup for GUI

The GUI depends on the game library SDL 2.0. To install it and set it up (on Ubuntu), use the following commands in a terminal.
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
The demo uses the GUI and therefore requires to setup SDL as described above. To build and run the demo, run the following commands in a terminal.
```
cd path/to/rds
make demo
./build/demo
```
# RDS
The Restrained Dynamical System (RDS) is a control method that avoids collisions for a wheeled vehicle. This repository contains (or will contain) the method's implementation, tools that allow to visualize and monitor its operation, a ROS interface, and example code and instructions to integrate, try out and demonstrate the functionality of the full system or its individual components.

## Software Structure

![Alt text](/docs/software.png?raw=true "Each box corresponds to one source folder.")

## Visualization Setup

The visualization depends on the game library SDL 2.0. To install it and set it up (on Ubuntu), use the following commands in a terminal.
```
sudo apt install libsdl2-dev
```

## Demos
The demos use the visualization and therefore requires to setup SDL as described above. To build and run the demos, run the following commands in a terminal.
```
cd [THIS_REPOSITORY_ROOT_FOLDER]/rds
make
./build/demo_geo
./build/demo_simu 3 s
```

## Install and run on ROS
```
sudo apt install ros-kinetic-velodyne-msgs ros-kinetic-velodyne-pointcloud
cd [THIS_REPOSITORY_ROOT_FOLDER]/rds
make demo_rds # this creates the necessary library files inside build/
cd [YOUR_CATKIN_WORKSPACE]
catkin build rds_ros
```

To run the rds node
```
rosrun rds_ros rds_ros_node
```
rds_ros_node listens to the topic /laserscan (sensor_msgs/LaserScan).

To run the gui node
```
rosrun rds_gui_ros rds_gui_ros_node
```

## Holonomic mode

The two ros nodes to use for holonomic robots are "rds_holonomic_ros_node" and "rds_holonomic_gui_ros_node".

When calling the service from the holonomic node, one needs to follow the convention that the linear velocity is treated as the cartesian forward velocity (y) and the angular velocity is treated as the cartesian lateral velocity (x, in robot fixed coordinates).

## Velocity limits
When calling the service there is also the need to specify the velocity and acceleration limits.
The hexagon limits for the velocity are explained by the sketch here:
https://github.com/epfl-lasa/rds/blob/master/docs/hexagon_limits.pdf

In the holonomic case, the omega axis is interpreted as the x-axis, i.e. the lateral component of the velocity in m/s. One can set a rectangular limitation by setting all the omega limits to the same value. The acceleration limits will be interpreted similarly (angular=x, linear=y) and the command_cycle_time value is used to generate velocity limits out of the acceleration limits (setting a very high value effectively disables the acceleration limits).

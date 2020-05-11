# RDS

Redirecting Driver Support (RDS) is a method to enable robots to reactively avoid imminent collisions with moving objects. This repository contains the RDS method's basic implementation, a ROS node to integrate it into a control loop via ROS, and another ROS node to monitor the first node.

## Qolo Setup 

```
mkdir -p catkin_ws_rds/src
cd catkin_ws_rds/src
git clone https://github.com/epfl-lasa/rds.git
cd rds/rds
make qolo_rds
cd ../../..
catkin build rds_ros
```

To run the RDS node:

```
rosrun rds_ros rds_ros_node
```

## GUI Setup

There is a GUI for monitoring on a desktop computer how RDS processes sensor data and computes the new velocity command from the nominal command. It depends on the game library SDL 2.0, which the following command installs (on Ubuntu).
```
sudo apt install libsdl2-dev
```
Then, one can setup the GUI as follows.

```
mkdir -p catkin_ws_rds/src
cd catkin_ws_rds/src
git clone https://github.com/epfl-lasa/rds.git
cd rds/rds
make gui
cd ../../..
catkin build rds_gui_ros
```

To run the GUI node:

```
rosrun rds_gui_ros rds_gui_ros_node
```
## Third parties acknowledgements

This repo contains the spline library from [this project](https://github.com/ttk592/spline), and thus it must inherit the GPL 2 license (if ever released). Further, it contains pedestrian trajectory data from [here](https://graphics.cs.ucy.ac.cy/research/downloads/crowd-data).

## Software Structure

![Alt text](/docs/software.png?raw=true "Each box corresponds to one source folder.")

## Holonomic mode (OUTDATED)

The two ros nodes to use for holonomic robots are "rds_holonomic_ros_node" and "rds_holonomic_gui_ros_node".

When calling the service from the holonomic node, one needs to follow the convention that the linear velocity is treated as the cartesian forward velocity (y) and the angular velocity is treated as the cartesian lateral velocity (x, in robot fixed coordinates).

## Velocity limits
When calling the service there is also the need to specify the velocity and acceleration limits.
The hexagon limits for the velocity are explained by the sketch here:
https://github.com/epfl-lasa/rds/blob/master/docs/hexagon_limits.pdf

In the holonomic case, the omega axis is interpreted as the x-axis, i.e. the lateral component of the velocity in m/s. One can set a rectangular limitation by setting all the omega limits to the same value. The acceleration limits will be interpreted similarly (angular=x, linear=y) and the command_cycle_time value is used to generate velocity limits out of the acceleration limits (setting a very high value effectively disables the acceleration limits).

## Demos for the ROS nodes (OUTDATED)
```
roscore
rosbag play -l [THIS_REPOSITORY_ROOT_FOLDER]/data/ethz_lrf_snippet.bag /sick_laser_front/cropped_scan:=/laserscan
rosrun rds_gui_ros rds_gui_ros_node
rosrun rds_ros rds_ros_node
python [THIS_REPOSITORY_ROOT_FOLDER]/rds_ros/scripts/rds_client_ros_node.py
```

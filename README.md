# Reactive Driving Support

This is the repository of Reactive Driving Support (RDS), a method for robots to avoid imminent collisions with moving obstacles, which has been developed as part of research at LASA, EPFL. This accompanying short [video](https://www.youtube.com/watch?v=RAKAhTWd7jw) explains the method. The corresponding publication is under review. This work was funded by the EU H2020 project [Crowdbot](http://crowdbot.eu/).

The repository hosts code that implements RDS, simulates and evaluates navigation with RDS in a crowd of pedestrians, and provides an interface for executing RDS on a robot using [ROS](https://www.ros.org/). The original code in this repository (everything except the following third parties' contributions) is subject to the MIT [LICENSE](LICENSE).

## Third parties acknowledgement

The folder spline contains the spline library from [this project](https://github.com/ttk592/spline) which is subject to the GPL 2 license. Further, the folder rds/data contains pedestrian trajectory data from [here](https://graphics.cs.ucy.ac.cy/research/downloads/crowd-data.html). Finally, the folder RVO2 contains a modified version of the [RVO2 library](https://github.com/snape/RVO2) which is subject to the Apache 2 license.

# Getting started with RDS

Instructions follow to set up and run RDS and some demos in Ubuntu 16.04. Below, the first section is for simulations, and the second section is for the ROS interface. First, the following steps are necessary in both cases.

This installs the graphics library SDL2, which is a prerequisite.

```
sudo apt install libsdl2-dev
```

This gets the code and builds the basic components for RDS.

```
git clone https://github.com/epfl-lasa/rds.git
cd rds/RVO2
mkdir build
cd build
cmake ..
make
sudo make install
cd ../../rds
make
```

## Simulations

The previous steps built the executable baseline_crowd_rds_5 under [THIS_REPOSITORY_ROOT_FOLDER]/rds/build/. The executable is for simulating a robot using RDS in a crowd whose individual pedestrians track reference trajectories from a real-world [dataset](https://graphics.cs.ucy.ac.cy/research/downloads/crowd-data.html).

For running the executable, one needs to be in the right folder:

```
cd [THIS_REPOSITORY_ROOT_FOLDER]/rds
```

The following command runs three simulations, one where there is no robot and two where the robot replaces the 7th pedestrian and tracks its reference trajectory, once using ORCA and once using RDS.

```
./build/baseline_crowd_rds_5 7
```

One can replace 7 by any integer i with 0 <= i < 430.

The following command runs the simulations for all configurations (with a gui) and saves the results of the evaluation as metrics_evaluation.csv.

```
./build/baseline_crowd_rds_5 gui
```

The following command runs the simulations for all configurations (without a gui) and saves the results of the evaluation as metrics_evaluation.csv.

```
./build/baseline_crowd_rds_5
```

## ROS interface

RDS can be integrated with a robot's perception and control system by using [ROS](https://www.ros.org/) and the nodes from this repository. The following sections show how to build and use these nodes in a ROS environment.

### Setup

As prerequisites, one needs to install [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) and the [catkin command line tools](https://catkin-tools.readthedocs.io/en/latest/installing.html) as the links describe. The next steps (below) are to create a catkin workspace and to build there this repository's nodes.

```
mkdir -p catkin_ws_rds/src
cd catkin_ws_rds/src
ln -s path/to/rds rds
cd ..
catkin init
catkin build --cmake-args -DRDS_ROS_USE_TRACKER=OFF
```

For integrating the people tracker of [Crowdbot](http://crowdbot.eu/), the package frame_msgs must be in the catkin workspace. The node for RDS can then listen to frame_msgs::TrackedPersons messages and treat the detections as obstacles. The following command builds the node accordingly (requires frame_msgs package).

```
catkin build
```

### Demo

The above steps have built the executables of the following four nodes. 

The rds_ros_node offers a service which execute RDS, listens to laserscan (and optionally tracker) messages, and publishes its operations as GUI messages.

The rds_gui_ros_node executes a GUI, listens to GUI messages and displays them.

The rds_ros_nominal_command_node calls the service which executes RDS from the rds_ros_node. It calls this service using a constant nominal command and is just for demonstration and debugging purposes.

The rds_ros_fake_lrf_node publishes two fake laserscan messages representing a front and rear sensor. It is just for demonstration and debugging purposes.

The following commands run the nodes together and demonstrate the operation of RDS in ROS. 

```
cd catkin_ws_rds
. devel/setup.bash
roslaunch rds_ros demo_standalone.launch
```

# OLD STUFF

Redirecting Driver Support (RDS) is a method for robots to reactively avoid imminent collisions with moving objects. This repository contains the RDS method's implementation, a ROS node to integrate it into a control loop via ROS, and another ROS node to monitor the first node. Further, it contains simulations of a capsule-shaped robot navigating in crowds and using the RDS method for local collision avoidance.

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

## Simulation Setup

The simulations require a modified version of the RVO2 library, which is installed by the following commands.
```
cd [THIS_REPOSITORY_ROOT_FOLDER]/RVO2
mkdir build
cd build
cmake ..
make
sudo make install
```
The simulation graphics further require installing SDL2 as described above. Then, the following commands build and run two example simulations.
```
cd [THIS_REPOSITORY_ROOT_FOLDER]/rds/rds
make build/dynamic_scene_complex_rds_5
make build/static_scene_rds_5
build/dynamic_scene_complex_rds_5
build/static_scene_rds_5
```

## Third parties acknowledgements

The folder spline contains the spline library from [this project](https://github.com/ttk592/spline) which is subject to the GPL 2 license. Further, the folder rds/data contains pedestrian trajectory data from [here](https://graphics.cs.ucy.ac.cy/research/downloads/crowd-data). Finally, the folder RVO2 contains a modified version of the [RVO2 library](https://github.com/snape/RVO2) which is subject to the Apache 2 license.

## Software Structure

![Alt text](/docs/software_2.png?raw=true "Each box corresponds to one source folder.")

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

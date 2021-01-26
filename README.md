# Reactive Driving Support

This is the repository of Reactive Driving Support (RDS), a method for robots to avoid imminent collisions with moving obstacles, which has been developed as part of research at LASA, EPFL. This accompanying short [video](https://www.youtube.com/watch?v=RAKAhTWd7jw) explains the method. The corresponding publication is under review. This work was funded by the EU H2020 project [Crowdbot](http://crowdbot.eu/).

## Software Structure

![Alt text](/docs/software_2.png?raw=true "Each box corresponds to one source folder.")

The repository hosts code that implements RDS, simulates and evaluates navigation with RDS in a crowd of pedestrians, and provides an interface for executing RDS on a robot using [ROS](https://www.ros.org/). The original code in this repository (everything except the following third parties' contributions) is subject to the MIT [LICENSE](LICENSE).

## Third parties acknowledgement

The folder spline contains the spline library from [this project](https://github.com/ttk592/spline) which is subject to the GPL 2 license. Further, the folder rds/data contains pedestrian trajectory data from [here](https://graphics.cs.ucy.ac.cy/research/downloads/crowd-data.html). Finally, the folder RVO2 contains a modified version of the [RVO2 library](https://github.com/snape/RVO2) which is subject to the Apache 2 license.

# Integrating RDS on a Robot

![Alt text](/docs/integration_steps.png?raw=true "...haha...")

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
ln -s [THIS_REPOSITORY_ROOT_FOLDER] rds
cd ..
catkin init
catkin build --cmake-args -DRDS_ROS_USE_TRACKER=OFF
```

For integrating the people tracker of [Crowdbot](http://crowdbot.eu/), the package frame_msgs must be in the catkin workspace. The node for RDS can then listen to frame_msgs::TrackedPersons messages and treat the detections as obstacles. The following command builds the node accordingly (requires the external package frame_msgs).

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

### RDS ROS service parameters

Using RDS via the ROS interface means sending a request for the service which the rds_ros_node offers (as done by [this node in python](rds_ros/scripts/rds_client_ros_node.py) and [this node in C++](rds_ros/src/nominal_command_node.cpp) from the demo). The file [VelocityCommandCorrectionRDS.srv](rds_network_ros/srv/VelocityCommandCorrectionRDS.srv) defines this service's layout in terms of the parameters which the service's request and its response specify. The request's parameters are passed as arguments to RDS, whereas the response's parameters store the results which RDS computes. The list below explains each parameters' meaning for RDS.

The request's parameter nominal_command specifies the nominal linear and angular velocity command which one issues for the robot but which does not yet take into account obstacles.

The robot's footprint is bounded by a capsule for RDS, whose geometry is specified by the parameters capsule_center_front_y, capsule_center_rear_y and capsule_radius. The reference_point_y denotes the point whose velocity is optimized to minimize the absolute difference from the point's nominal velocity. The definitions are according to the following sketch.

![Alt text](/docs/capsule_footprint.png?raw=true "this text is not shown.")

The parameter rds_tau specifies the time horizon which RDS uses to construct velocity obstacles.

The parameter rds_delta specifies the margin which RDS adds to each obstacle's radius to be more conservative.

The following sketch explains the parameters that specify constant limits for the linear and angular velocity (with their names on the right).

![Alt text](/docs/velocity_limits.png?raw=true "this text is not shown.")

The parameters acc_limit_linear_abs_max and acc_limit_angular_abs_max specify the linear and angular acceleration limits, respectively. To enforce them, the rds_ros_node relies on the next parameter dt, which should specify the time i.e. period between sequential requests. Then, the rds_ros_node converts the acceleration limits into velocity limits that are centered around the velocity which RDS computed in the previous cycle and enclose the velocities which are reachable within dt while satisfying the specified acceleration limits. 

The boolean parameter lrf_point_obstacles specifies whether to take laserscan points directly into account as obstacles or not (which is the only way for RDS to perceive obstacles without a tracker).

The boolean parameter ORCA_implementation specifies whether to use an implementation of ORCA instead of RDS (for comparison purposes).

The response's parameter corrected_command reports the command which RDS has computed.

The response's parameter call_counter simply counts how often the service of the rds_ros_node has been called since the node was launched.
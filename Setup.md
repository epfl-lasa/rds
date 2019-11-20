# Setup

First, build the rds core functions.
```
cd path/to/rds
cd rds
make rds_wrap
```
Second, create a catkin workspace and build the ros nodes which use the rds core functions.
```
sudo apt-get install ros-kinetic-velodyne # if necessary
mkdir -p ~/catkin_rds/src
echo ". ~/catkin_rds/devel/setup.bash" >> ~/.bashrc
cd ~/catkin_rds/src
ln -s path/to/rds
cd ..
catkin_make
```
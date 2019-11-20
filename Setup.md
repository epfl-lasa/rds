# Setup

```
cd path/to/rds
cd rds
make rds_wrap
```

```
mkdir -p ~/catkin_rds/src
echo ". ~/catkin_rds/devel/setup.bash" >> ~/.bashrc
cd ~/catkin_rds/src
ln -s path/to/rds
cd ..
catkin_make
```

cd src/rds/rds
make rds_wrap
cd -
rm -r build/rds
rm -r src/rds/rds_gui_ros
catkin_make

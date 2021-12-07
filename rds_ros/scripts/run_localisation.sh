#!/bin/bash

#----- Colored Terminal -----
NORMAL="\e[0m"
IMP_INFO="\e[34;1m"
IMP_RED="\e[31;1m"
IMP_GREEN="\e[32;1m"

#----- Ctrl-C stop -----
_kill() {
    echo -e "${IMP_RED}Killing all subprocesses${NORMAL}"
    for PID in ${PID_LIST[@]};do
        kill -INT $PID
    done
}
trap "exit" INT TERM ERR
trap _kill EXIT

#----- Launch Front Lidar  -----
echo -e "${IMP_INFO}Launching Velodyne Frontal LIDAR...${NORMAL}"
eval ". ~/autonomy_ws/devel/setup.bash"
eval "roslaunch rds_ros front_lrf_new_driver.launch &"
PID_LIST+="$! "

# eval ". /home/qolo/hasler_ws/devel/setup.bash"
# echo "Launching SLAM..."
# eval "roslaunch hector_mapping mapping_qolo.launch"
sleep 5

#----- Launch laser_scan_matcher + IMU -----
echo -e "${IMP_INFO}Launching Laser Scan Matcher...${NORMAL}"
eval ". ~/localization_ws/devel/setup.bash"
eval "rosrun openzen_sensor openzen_sensor_node &"
sleep 2
eval "roslaunch ~/localization_ws/laser_scan_matcher.launch &"
PID_LIST+="$! "
sleep 5


# Wait till all pids to be finished or killed
echo "All PIDs : ${PID_LIST}"
for PID in ${PID_LIST[@]};do
    wait $PID
done

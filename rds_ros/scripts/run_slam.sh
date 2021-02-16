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

#----- Launch Rear Lidar  -----
echo -e "${IMP_INFO}Launching Velodyne Frontal LIDAR...${NORMAL}"
eval ". ~/autonomy_ws/devel/setup.bash"
eval "roslaunch rds_ros rds_front_lrf_slam.launch"
# eval ". /home/qolo/hasler_ws/devel/setup.bash"
# echo "Launching SLAM..."
# eval "roslaunch hector_mapping mapping_qolo.launch"
PID_LIST+="$! "

sleep 5


# Wait till all pids to be finished or killed
echo "All PIDs : ${PID_LIST}"
for PID in ${PID_LIST[@]};do
    wait $PID
done

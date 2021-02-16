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

#----- Get Test Number -----
TEST_NO=0
#while [ -d "csv_logs/test${TEST_NO}" ]; do
#    TEST_NO=$(( $TEST_NO + 1 ))
#done
#LOG_FOLDER="$(pwd)/csv_logs/test${TEST_NO}"
#eval "mkdir -p ${LOG_FOLDER}"
# echo "Current Test Number : ${TEST_NO}"
eval ". devel/setup.bash"

#----- Launch and record force sensors -----
#eval "./src/rokubimini_interface/run_rokubimini_ros.sh -f ${LOG_FOLDER}&"
#PID_LIST+="$! "

#sleep 5

#----- Launch and record realsense camera -----
# echo "Launching RealSense Camera..."
# eval "roslaunch realsense2_camera rs_minimum.launch \
#     &> /dev/null &"
# PID_LIST+="$! "

# #eval "rosbag record -q \
# #    -O ${LOG_FOLDER}/camera \
# #    -e '/camera_front/(.*)' \
# #    &> /dev/null &"
# PID_LIST+="$! "

# sleep 5

#----- Launch Rear Lidar  -----
echo "Launching Velodyne Frontal LIDAR..."
eval "roslaunch rds_ros rds_front_lrf_cloud.launch"
# eval ". /home/qolo/hasler_ws/devel/setup.bash"
# echo "Launching SLAM..."
# eval "roslaunch hector_mapping mapping_qolo.launch"
PID_LIST+="$! "

#eval "rosrun tf static_transform_publisher 0 0 0 0 0 0 1 /tf_qolo camera_front_color_optical_frame 10"
sleep 5

# Wait till all pids to be finished or killed
echo "All PIDs : ${PID_LIST}"
for PID in ${PID_LIST[@]};do
    wait $PID
done
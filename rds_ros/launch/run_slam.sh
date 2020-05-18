#!/bin/bash

#----- Ctrl-C stop -----
trap "exit" INT TERM ERR
trap "kill -INT 0" EXIT

#----- Get Test Number -----
TEST_NO=0
#while [ -d "csv_logs/test${TEST_NO}" ]; do
#    TEST_NO=$(( $TEST_NO + 1 ))
#done
#LOG_FOLDER="$(pwd)/csv_logs/test${TEST_NO}"
#eval "mkdir -p ${LOG_FOLDER}"
# echo "Current Test Number : ${TEST_NO}"


#----- Launch and record force sensors -----
#eval "./src/rokubimini_interface/run_rokubimini_ros.sh -f ${LOG_FOLDER}&"
#PID_LIST+="$! "

#sleep 5

# #----- Launch and record realsense camera -----
# echo "Launching RealSense Camera..."
# eval "roslaunch realsense2_camera rs_minimum.launch \
#     &> /dev/null &"
# PID_LIST+="$! "

#----- Launch Rear Lidar  -----
echo "Launching Velodyne Frontal LIDAR..."
eval ". devel/setup.bash"
eval "roslaunch rds_ros rds_slam.launch"
# eval ". /home/qolo/hasler_ws/devel/setup.bash"
# echo "Launching SLAM..."
# eval "roslaunch hector_mapping mapping_qolo.launch"
PID_LIST+="$! "

#eval "rosrun tf static_transform_publisher 0 0 0 0 0 0 1 /tf_qolo camera_front_color_optical_frame 10"
sleep 5

# #eval "rosbag record -q \
# #    -O ${LOG_FOLDER}/camera \
# #    -e '/camera_front/(.*)' \
# #    &> /dev/null &"
# PID_LIST+="$! "

# sleep 5

# Wait till all pids to be finished or killed
echo "All PIDs : ${PID_LIST}"
for PID in ${PID_LIST[@]};do
    wait $PID
done

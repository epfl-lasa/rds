# source this in the eval directory with bag files as arguments (but first launch the roscore) 
for var in "$@"
do
    echo "$var"
    python bag_to_mat_node.py &
    this_PID=$!
    rosbag play "$var"
    kill -2 $this_PID
    sleep 5
    mv commands_log.mat "$var"_commands_log.mat
    mv "$var"_commands_log.mat ./    
    mv collision_points_log.mat "$var"_collision_points_log.mat
    mv "$var"_collision_points_log.mat ./
done

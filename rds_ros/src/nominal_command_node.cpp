#include <rds_network_ros/VelocityCommandCorrectionRDS.h>

#include <ros/ros.h>

#include <chrono>
#include <thread>

int main(int argc, char** argv)
{
	rds_network_ros::VelocityCommandCorrectionRDS srv;
	rds_network_ros::VelocityCommandCorrectionRDS::Request& request = srv.request;

	request.nominal_command.linear = 0.5;
	request.nominal_command.angular = 0.2;

	ros::init(argc, argv, "rds_ros_nominal_command_node");
	ros::NodeHandle n;
	ros::ServiceClient c(n.serviceClient<rds_network_ros::VelocityCommandCorrectionRDS>("rds_velocity_command_correction"));

	std::chrono::milliseconds command_cycle_time(10);

	std::chrono::high_resolution_clock::time_point t1, t2;
	while (ros::ok())
	{
		t1 = std::chrono::high_resolution_clock::now();
		c.call(srv);
		ROS_INFO("Nominal command [%f,%f], corrected command [%f,%f], got_it=%d",
			request.nominal_command.linear, request.nominal_command.angular,
			srv.response.corrected_command.linear, srv.response.corrected_command.angular,
			srv.response.got_it);
		ros::spinOnce();
		t2 = std::chrono::high_resolution_clock::now();
		std::this_thread::sleep_for(command_cycle_time - std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1));
	}

	return 0;
}

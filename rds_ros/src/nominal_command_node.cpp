#include <rds/differential_drive_kinematics.hpp>

#include <rds_network_ros/VelocityCommandCorrectionRDS.h>

#include <ros/ros.h>

#include <chrono>
#include <thread>

int main(int argc, char** argv)
{
	RDS::VelocityCommand nominal_command(0.5f, 0.f);

	RDS::VelocityCommandHexagonLimits hexagon_limits;
	hexagon_limits.min_linear = -0.75f;
	hexagon_limits.max_linear = 1.75f;
	hexagon_limits.absolute_angular_at_min_linear = 0.f;
	hexagon_limits.absolute_angular_at_max_linear = 0.f;
	hexagon_limits.absolute_angular_at_zero_linear = 1.f;

	RDS::VelocityCommandBoxLimits box_limits;
	box_limits.min_linear = -10.5f;
	box_limits.min_angular = -10.25f;
	box_limits.max_linear = 11.5f;
	box_limits.max_angular = 10.75f;//these high values are just to disable the limits for now

	rds_network_ros::VelocityCommandCorrectionRDS srv;
	rds_network_ros::VelocityCommandCorrectionRDS::Request& request = srv.request;

	request.nominal_command.linear = nominal_command.linear;
	request.nominal_command.angular = nominal_command.angular;

	request.velocity_limits.min_linear = hexagon_limits.min_linear;
	request.velocity_limits.max_linear = hexagon_limits.max_linear;
	request.velocity_limits.abs_angular_at_min_linear = hexagon_limits.absolute_angular_at_min_linear;
	request.velocity_limits.abs_angular_at_max_linear = hexagon_limits.absolute_angular_at_max_linear;
	request.velocity_limits.abs_angular_at_zero_linear = hexagon_limits.absolute_angular_at_zero_linear;

	request.last_actual_command.linear = (box_limits.min_linear + box_limits.max_linear)/2.f;
	request.last_actual_command.angular = (box_limits.min_angular + box_limits.max_angular)/2.f;
	request.command_cycle_time = 0.1f;
	request.abs_linear_acceleration_limit = (box_limits.max_linear - box_limits.min_linear)/2.f/request.command_cycle_time;
	request.abs_angular_acceleration_limit = (box_limits.max_angular - box_limits.min_angular)/2.f/request.command_cycle_time;

	ros::init(argc, argv, "rds_ros_nominal_command_node");
	ros::NodeHandle n;
	ros::ServiceClient c(n.serviceClient<rds_network_ros::VelocityCommandCorrectionRDS>("velocity_command_correction_rds"));

	std::chrono::milliseconds command_cycle_time(int(request.command_cycle_time*1000.f));

	std::chrono::high_resolution_clock::time_point t1, t2;
	while (ros::ok())
	{
		t1 = std::chrono::high_resolution_clock::now();
		c.call(srv);
		ros::spinOnce();
		t2 = std::chrono::high_resolution_clock::now();
		std::this_thread::sleep_for(command_cycle_time - std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1));
	}

	return 0;
}

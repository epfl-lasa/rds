#include "wrapper_base.hpp"

#include <rds_network_ros/VelocityCommandCorrectionRDS.h>

#include <ros/ros.h>

#include <chrono>
#include <thread>

int main(int argc, char** argv)
{
	WrapperBase calling_wrapper("config/qolo");
	const float nominal_command_linear = 1.0;
	const float nominal_command_angular = 0.5;
	float corrected_command_linear, corrected_command_angular;
	/*
	rds_network_ros::VelocityCommandCorrectionRDS srv;
	rds_network_ros::VelocityCommandCorrectionRDS::Request& request = srv.request;

	request.nominal_command.linear = 1.0;
	request.nominal_command.angular = 0.5;
	request.capsule_center_front_y = 0.18;
	request.capsule_center_rear_y = -0.5;
	request.capsule_radius = 0.45;
	request.reference_point_y = 0.18;
	request.rds_tau = 1.5;
	request.rds_delta = 0.05;
	request.vel_lim_linear_min = -0.5;
	request.vel_lim_linear_max = 1.5;
	request.vel_lim_angular_abs_max = 1.0;
	request.vel_linear_at_angular_abs_max = 0.2;
	request.acc_limit_linear_abs_max = 0.5;
	request.acc_limit_angular_abs_max = 0.5;
	request.dt = 0.01;

	// shall rds consider lrf measurements?
	request.lrf_point_obstacles = true;

	// execute the baseline method (ORCA-like) instead of RDS?
	request.ORCA_implementation = false;

	ros::init(argc, argv, "rds_ros_nominal_command_node");
	ros::NodeHandle n;
	ros::ServiceClient c(n.serviceClient<rds_network_ros::VelocityCommandCorrectionRDS>("rds_velocity_command_correction"));
	*/
	std::chrono::milliseconds command_cycle_time(10);

	std::chrono::high_resolution_clock::time_point t1, t2;
	while (ros::ok())
	{
		t1 = std::chrono::high_resolution_clock::now();
		if (calling_wrapper.callRDS(nominal_command_linear, nominal_command_angular,
			&corrected_command_linear, &corrected_command_angular) == 0)
		{
			ROS_INFO("Nominal command [%f,%f], corrected command [%f,%f]",
				nominal_command_linear, nominal_command_angular,
				corrected_command_linear, corrected_command_angular);
		}
		/*c.call(srv);
		ROS_INFO("Nominal command [%f,%f], corrected command [%f,%f], call_counter=%i",
			request.nominal_command.linear, request.nominal_command.angular,
			srv.response.corrected_command.linear, srv.response.corrected_command.angular,
			srv.response.call_counter);
		*/
		ros::spinOnce();
		t2 = std::chrono::high_resolution_clock::now();
		std::this_thread::sleep_for(command_cycle_time - std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1));
	}

	return 0;
}

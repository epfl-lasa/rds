#include <rds_network_ros/VelocityCommandCorrectionRDS.h>

#include <ros/ros.h>

#include <chrono>
#include <thread>

int main(int argc, char** argv)
{
	rds_network_ros::VelocityCommandCorrectionRDS srv;
	rds_network_ros::VelocityCommandCorrectionRDS::Request& request = srv.request;

	request.nominal_command.linear = 1.0;
	request.nominal_command.angular = 0.5;
	request.capsule_center_front_y = 0.05;
	request.capsule_center_rear_y = -0.5;
	request.capsule_radius = 0.45;
	request.reference_point_y = 0.25;
	request.rds_tau = 1.5;
	request.rds_delta = 0.05;
	request.vel_lim_linear_min = 0.5;
	request.vel_lim_linear_max = 1.5;
	request.vel_lim_angular_abs_max = 1.0;
	request.vel_linear_at_angular_abs_max = 0.2;
	request.acc_limit_linear_abs_max = 0.5;
	request.acc_limit_angular_abs_max = 0.5;
	request.dt = 1.0/15.0;

	// shall rds consider lrf measurements?
	request.lrf_point_obstacles = true;

	// for generating constraints due to raw lrf scan points,
	// shall rds use the VO-based or the alternative (prior) approach?
	request.lrf_alternative_rds = false;

	// how shall rds choose the base velocity for determining the convex approximate VO?
	// 0 : use zero velocity (ensures that the final halfplane contains the VO, if the VO does not contain the origin)
	// 1 : use the velocity which rds computed previously
	// any other integer: use the nominal velocity (from the current nominal command)
	request.vo_tangent_base_command = 2;

	// shall rds map the base velocity to the tangent point the same way as ORCA for determining the convex approximate VO?
	request.vo_tangent_orca_style = false;

	// shall rds work with bounding circles or find per object the closest incircle in the capsule?
	// any integer n > 2 : use n bounding circles
	// any integer <= 2 : use local capsule incircles
	request.bounding_circles = 0;

	ros::init(argc, argv, "rds_ros_nominal_command_node");
	ros::NodeHandle n;
	ros::ServiceClient c(n.serviceClient<rds_network_ros::VelocityCommandCorrectionRDS>("rds_velocity_command_correction"));

	std::chrono::milliseconds command_cycle_time(10);

	std::chrono::high_resolution_clock::time_point t1, t2;
	while (ros::ok())
	{
		t1 = std::chrono::high_resolution_clock::now();
		c.call(srv);
		ROS_INFO("Nominal command [%f,%f], corrected command [%f,%f], call_counter=%i",
			request.nominal_command.linear, request.nominal_command.angular,
			srv.response.corrected_command.linear, srv.response.corrected_command.angular,
			srv.response.call_counter);
		ros::spinOnce();
		t2 = std::chrono::high_resolution_clock::now();
		std::this_thread::sleep_for(command_cycle_time - std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1));
	}

	return 0;
}

#include <rds_ros/wrapper_holonomic.hpp>
#include <rds_network_ros/VelocityCommandCorrectionRDS.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <string>
#include <chrono>
#include <thread>

int main(int argc, char** argv)
{
	std::string config_path_local = "config/default_holonomic";
	std::string package_path = ros::package::getPath("user_pkg_example");
	std::string config_path = package_path + "/" + config_path_local;
	WrapperHolonomic calling_wrapper(config_path, 0, false);
	const float nominal_command_vx = 1.0;
	const float nominal_command_vy = 0.5;
	float corrected_command_vx, corrected_command_vy;

	std::chrono::milliseconds command_cycle_time(100);

	std::chrono::high_resolution_clock::time_point t1, t2;
	while (ros::ok())
	{
		t1 = std::chrono::high_resolution_clock::now();
		if (calling_wrapper.callRDS(nominal_command_vx, nominal_command_vy,
			&corrected_command_vx, &corrected_command_vy) == 0)
		{
			ROS_INFO("Nominal command [%f,%f], corrected command [%f,%f]",
				nominal_command_vx, nominal_command_vy,
				corrected_command_vx, corrected_command_vy);
		}
		ros::spinOnce();
		t2 = std::chrono::high_resolution_clock::now();
		std::this_thread::sleep_for(command_cycle_time - std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1));
	}

	return 0;
}

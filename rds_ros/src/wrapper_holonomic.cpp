#include "wrapper_holonomic.hpp"

WrapperHolonomic::WrapperHolonomic(const std::string& config_filepath,
	ros::NodeHandle* existing_node_handle)
  : WrapperBase("", existing_node_handle)
  , virtual_center_y(0.1f)
{
	std::string package_path = ros::package::getPath("rds_ros");

	std::ifstream file(package_path + "/" + config_filepath);
	if (file)
	{
		float radius = readFloat(
			getValueForKeyFromConfigFile(file, "radius"), 0.5f);
		service_form.request.rds_tau = readFloat(
			getValueForKeyFromConfigFile(file, "rds_tau"), 1.f);
		service_form.request.rds_delta = readFloat(
			getValueForKeyFromConfigFile(file, "rds_delta"), 0.05f);
		float vel_lim_x_abs_max = readFloat(
			getValueForKeyFromConfigFile(file, "vel_lim_x_abs_max"), 1.f);
		float vel_lim_y_min = readFloat(
			getValueForKeyFromConfigFile(file, "vel_lim_y_min"), -1.f);
		float vel_lim_y_max = readFloat(
			getValueForKeyFromConfigFile(file, "vel_lim_y_max"), 1.f);	
		float acc_limit_x_abs_max = readFloat(
			getValueForKeyFromConfigFile(file, "acc_limit_x_abs_max"), 1.f);	
		float acc_limit_y_abs_max = readFloat(
			getValueForKeyFromConfigFile(file, "acc_limit_y_abs_max"), 1.f);	
		service_form.request.dt = readFloat(
			getValueForKeyFromConfigFile(file, "dt"), 0.1f);
		service_form.request.lrf_point_obstacles = readBool(
			getValueForKeyFromConfigFile(file, "lrf_point_obstacles"), true);	
		service_form.request.ORCA_implementation = readBool(
			getValueForKeyFromConfigFile(file, "ORCA_implementation"), false);
		file.close();

		service_form.request.capsule_center_front_y = virtual_center_y;
		service_form.request.capsule_center_rear_y = virtual_center_y;
		service_form.capsule_radius = radius;
		service_form.reference_point_y = virtual_center_y;
		service_form.vel_lim_linear_min = vel_lim_y_min;
		service_form.vel_lim_linear_max = vel_lim_y_max;
		service_form.vel_linear_at_angular_abs_max = 0.f;
		service_form.vel_lim_angular_abs_max = vel_lim_x_abs_max/virtual_center_y;
		service_form.acc_limit_linear_abs_max = acc_limit_y_abs_max;
		service_form.acc_limit_angular_abs_max = acc_limit_x_abs_max/virtual_center_y;
	}
	else
		std::cout << "\033[1;31mCould not open config file:\033[0m" << config_filepath << "\n"; 
	return;
}

int WrapperHolonomic::callRDS(float vx_n, float vy_n, float* vx_c, float* vy_c)
{
	float v_n = vy_n;
	float w_n = -vx_n/virtual_center_y;
	float v_c, w_c;
	int error = WrapperBase::callRDS(v_n, w_n, &v_c, &w_c);
	*vx_c = -w_c*virtual_center_y;
	*vy_c = v_c;
	return error;
}
#include <rds_ros/wrapper_base.hpp>
#include <ros/package.h>
#include <fstream>

WrapperBase::WrapperBase(const std::string& config_filepath,
	ros::NodeHandle* existing_node_handle, bool local_path_rds_ros_pkg)
{
	if (existing_node_handle == 0)
	{
		int argc = 1;
		char argv1[10] = "dummy";
		char* argv[1] = {argv1};
		ros::init(argc, argv, "rds_ros_nominal_command_node");
		node_handle = new ros::NodeHandle();
	}
	else
	{
		node_handle = existing_node_handle;
	}
	rds_client = new ros::ServiceClient(
		node_handle->serviceClient<rds_network_ros::VelocityCommandCorrectionRDS>(
			"rds_velocity_command_correction"));
	call_counter = 0;

	std::string package_path = ros::package::getPath("rds_ros") + "/";
	if (!local_path_rds_ros_pkg)
		package_path = "";
	if (config_filepath == "")
		return;
	std::ifstream file(package_path + config_filepath);
	if (file)
	{
		service_form.request.capsule_center_front_y = readFloat(
			getValueForKeyFromConfigFile(file, "capsule_center_front_y"), 0.5f);
		service_form.request.capsule_center_rear_y = readFloat(
			getValueForKeyFromConfigFile(file, "capsule_center_rear_y"), -0.5f);
		service_form.request.capsule_radius = readFloat(
			getValueForKeyFromConfigFile(file, "capsule_radius"), 0.5f);
		service_form.request.reference_point_y = readFloat(
			getValueForKeyFromConfigFile(file, "reference_point_y"), 0.5f);
		service_form.request.rds_tau = readFloat(
			getValueForKeyFromConfigFile(file, "rds_tau"), 1.f);
		service_form.request.rds_delta = readFloat(
			getValueForKeyFromConfigFile(file, "rds_delta"), 0.05f);
		service_form.request.vel_lim_linear_min = readFloat(
			getValueForKeyFromConfigFile(file, "vel_lim_linear_min"), -1.f);
		service_form.request.vel_lim_linear_max = readFloat(
			getValueForKeyFromConfigFile(file, "vel_lim_linear_max"), 1.f);
		service_form.request.vel_lim_angular_abs_max = readFloat(
			getValueForKeyFromConfigFile(file, "vel_lim_angular_abs_max"), 1.f);
		service_form.request.vel_linear_at_angular_abs_max = readFloat(
			getValueForKeyFromConfigFile(file, "vel_linear_at_angular_abs_max"), 0.f);	
		service_form.request.acc_limit_linear_abs_max = readFloat(
			getValueForKeyFromConfigFile(file, "acc_limit_linear_abs_max"), 1.f);	
		service_form.request.acc_limit_angular_abs_max = readFloat(
			getValueForKeyFromConfigFile(file, "acc_limit_angular_abs_max"), 1.f);	
		service_form.request.dt = readFloat(
			getValueForKeyFromConfigFile(file, "dt"), 0.1f);
		service_form.request.lrf_point_obstacles = readBool(
			getValueForKeyFromConfigFile(file, "lrf_point_obstacles"), true);	
		service_form.request.ORCA_implementation = readBool(
			getValueForKeyFromConfigFile(file, "ORCA_implementation"), false);
		file.close();
	}
	else
		std::cout << "\033[1;31mCould not open config file:\033[0m" << config_filepath << "\n"; 
	//ROS_INFO("%f", service_form.request.capsule_radius);
	//ROS_INFO("%i", int(service_form.request.ORCA_implementation));
}

WrapperBase::~WrapperBase()
{
	delete rds_client;
	delete node_handle;
}

int WrapperBase::callRDS(float v_n, float w_n, float* v_c, float* w_c)
{
	rds_network_ros::VelocityCommandCorrectionRDS this_service_form(service_form);
	this_service_form.request.nominal_command.linear = v_n;
	this_service_form.request.nominal_command.angular = w_n;
	rds_client->call(this_service_form);
	if (this_service_form.response.call_counter > call_counter)
	{
		call_counter = this_service_form.response.call_counter;
		*v_c = this_service_form.response.corrected_command.linear;
		*w_c = this_service_form.response.corrected_command.angular;
		return 0;
	}
	else
		return 1;
}

std::string WrapperBase::getValueForKeyFromConfigFile(std::ifstream& config_file,
	const std::string& key)
{
	std::string result = "";
	std::string line;
	while( std::getline(config_file, line) )
	{
		//ROS_INFO("%s", line.c_str());
		std::istringstream is_line(line);
		std::string key_present;
		if( std::getline(is_line, key_present, '=') )
		{
			std::string value;
			if( std::getline(is_line, value) )
			{
				if (key_present == key)
					result = value;
			}
		}
	}
	config_file.clear();
	config_file.seekg(0);
	return result;
}

float WrapperBase::readFloat(const std::string& value, float default_value)
{
	try
	{
		return std::stof(value);
	}
	catch (...)
	{
		return default_value;
	}
}

bool WrapperBase::readBool(const std::string& value, bool default_value)
{
	if ((value == "1") || (value == "true") || (value == "True"))
		return true;
	else if ((value == "0") || (value == "false") || (value == "False"))
		return false;
	else
		return default_value;
}
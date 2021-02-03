#include "wrapper_base.hpp"
#include <fstream>

WrapperBase::WrapperBase(const std::string& config_filepath,
	ros::NodeHandle* existing_node_handle)
{
	if (existing_node_handle == 0)
	{
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

	std::ifstream file(config_filepath);
	if (file)
	{
		std::istringstream config_buffer;
		config_buffer << file.rdbuf();
		file.close();

		service_form.request.capsule_center_front_y = readFloat(
			getValueForKeyFromConfigFile(config_buffer, "capsule_center_front_y"), 0.5f);
		service_form.request.capsule_center_rear_y = readFloat(
			getValueForKeyFromConfigFile(config_buffer, "capsule_center_rear_y"), -0.5f);
		service_form.request.capsule_radius = readFloat(
			getValueForKeyFromConfigFile(config_buffer, "capsule_radius"), 0.5f);
		service_form.request.reference_point_y = readFloat(
			getValueForKeyFromConfigFile(config_buffer, "reference_point_y"), 0.5f);
		service_form.request.rds_tau = readFloat(
			getValueForKeyFromConfigFile(config_buffer, "rds_tau"), 1.f);
		service_form.request.rds_delta = readFloat(
			getValueForKeyFromConfigFile(config_buffer, "rds_delta"), 0.05f);
		service_form.request.vel_lim_linear_min = readFloat(
			getValueForKeyFromConfigFile(config_buffer, "vel_lim_linear_min"), -1.f);
		service_form.request.vel_lim_linear_max = readFloat(
			getValueForKeyFromConfigFile(config_buffer, "vel_lim_linear_max"), 1.f);
		service_form.request.vel_lim_angular_abs_max = readFloat(
			getValueForKeyFromConfigFile(config_buffer, "vel_lim_angular_abs_max"), 1.f);
		service_form.request.vel_linear_at_angular_abs_max = readFloat(
			getValueForKeyFromConfigFile(config_buffer, "vel_linear_at_angular_abs_max"), 0.f);	
		service_form.request.acc_limit_linear_abs_max = readFloat(
			getValueForKeyFromConfigFile(config_buffer, "acc_limit_linear_abs_max"), 1.f);	
		service_form.request.acc_limit_angular_abs_max = readFloat(
			getValueForKeyFromConfigFile(config_buffer, "acc_limit_angular_abs_max"), 1.f);	
		service_form.request.dt = readFloat(
			getValueForKeyFromConfigFile(config_buffer, "dt"), 0.1f);
		service_form.request.lrf_point_obstacles = readBool(
			getValueForKeyFromConfigFile(config_buffer, "lrf_point_obstacles"), true);	
		service_form.request.ORCA_implementation = readBool(
			getValueForKeyFromConfigFile(config_buffer, "ORCA_implementation"), false);
	}
	else
		std::cout << "\033[1;31mCould not open config file:\033[0m" << config_filepath << "\n"; 
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

static std::string WrapperBase::getValueForKeyFromConfigFile(const std::string& key,
	const std::istringstream& config_file);
{
	std::istringstream config_file_copy(config_file)
	std::string line;
	while( std::getline(config_file_copy, line) )
	{
		std::istringstream is_line(line);
		std::string key_present;
		if( std::getline(is_line, key_present, '=') )
		{
			std::string value;
			if( std::getline(is_line, value) )
				if (std::strcmp(key_present, key) == 0)
					return value;
		}
	}
	return "";
}

static float WrapperBase::readFloat(const std::string& value, float default_value)
{
	try
	{
		return std::stof(value);
	}
	catch (...)
		return default_value;
}

static bool WrapperBase::readBool(const std::string& value, bool default_value)
{
	switch (value)
	{
		case "1":
		{
			return true;
			break;
		}
		case "0":
		{
			return false;
			break;
		}
		case "True":
		{
			return true;
			break;
		}
		case "true":
		{
			return true;
			break;
		}
		case "False":
		{
			return false;
			break;
		}
		case "false":
		{
			return false;
			break;
		}
		default :
			return default_value;
	}
}
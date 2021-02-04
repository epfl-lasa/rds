#ifndef WRAPPER_BASE_HPP
#define WRAPPER_BASE_HPP

#include <rds_network_ros/VelocityCommandCorrectionRDS.h>
#include <ros/ros.h>
#include <iostream>
#include <string>

struct WrapperBase
{
	WrapperBase(const std::string& config_filepath = "config/default",
		ros::NodeHandle* existing_node_handle = 0, bool local_path_rds_ros_pkg = true);
	virtual ~WrapperBase();

	virtual int callRDS(float v_n, float w_n, float* v_c, float* w_c);
protected:
	static std::string getValueForKeyFromConfigFile(std::ifstream& config_file,
		const std::string& key);
	static float readFloat(const std::string& value, float default_value);
	static bool readBool(const std::string& value, bool default_value);

	ros::NodeHandle* node_handle;
	ros::ServiceClient* rds_client;
	unsigned int call_counter;
public:
	rds_network_ros::VelocityCommandCorrectionRDS service_form;
};

#endif
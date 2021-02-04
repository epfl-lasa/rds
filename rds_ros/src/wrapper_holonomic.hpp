#define WRAPPER_HOLONOMIC_HPP
#define WRAPPER_HOLONOMIC_HPP

#include "wrapper_base.hpp"

#include <ros/ros.h>
#include <string>

struct WrapperHolonomic : public WrapperBase
{
	WrapperHolonomic(const std::string& config_filepath = "config/default_holonomic",
		ros::NodeHandle* existing_node_handle = 0);

	virtual int callRDS(float vx_n, float vy_n, float* vx_c, float* vy_c);

private:
	const float virtual_center_y;
};

#endif
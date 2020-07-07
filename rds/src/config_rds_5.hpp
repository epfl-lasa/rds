#ifndef CONFIG_RDS_5_HPP
#define CONFIG_RDS_5_HPP

#include "geometry.hpp"
#include "rds_5_agent.hpp"

namespace ConfigRDS5
{
const float linear_acceleration_limit_jun_10 = 1.5;
const float angular_acceleration_limit_jun_10 = 1.5;
const float linear_acceleration_limit = 2.0;
const float angular_acceleration_limit = 3.0;

const VWDiamond vw_diamond_limits_jun_10(-0.75, 1.5, 4.124/3.5, 0.2);

const VWDiamond vw_diamond_limits_generous(-0.75, 1.5, 4.124, 0.2);

const VWDiamond vw_diamond_limits_very_generous(-1, 2, 4, 0);

const float robot_radius = 0.45;
const float y_center_front = 0.18;//0.051;//
const float y_center_back = -0.5;
const Geometry2D::Capsule robot_shape(robot_radius, Geometry2D::Vec2(0.0, y_center_front),
		Geometry2D::Vec2(0.0, y_center_back));
const float delta = 0.05;
const float tau = 1.5;
const float y_p_ref = 0.18;

struct ConfigWrap
{
	ConfigWrap(double dt) : rds_5_config(tau, delta, y_p_ref,
	linear_acceleration_limit, angular_acceleration_limit, dt,
	vw_diamond_limits_very_generous, robot_shape) { }

	const RDS5CapsuleConfiguration rds_5_config;
};
}

#endif
#ifndef RDS_WRAP_HPP
#define RDS_WRAP_HPP

#include "differential_drive_kinematics.hpp"
#include "collision_point.hpp"

RDS::VelocityCommand compute_RDS_command(const RDS::VelocityCommand& nominal_command,
	const RDS::VelocityCommandBoxLimits& box_limits,
	const RDS::VelocityCommandHexagonLimits& hexagon_limits,
	const std::vector<RDS::CollisionPoint>& collision_points,
	float y_coordinate_of_reference_point_for_command_limits = 0.5f,
	float weight_scaling_of_reference_point_for_command_limits = 1.f,
	float tau = 2.f,
	float delta = 0.1f,
	float clearance_from_axle_of_final_reference_point = 0.15f);

#endif
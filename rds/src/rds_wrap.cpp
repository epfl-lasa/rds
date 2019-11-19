#include "rds_wrap.hpp"

RDSWrap::RDSWrap(const RDS::VelocityCommand& nominal_command,
	const RDS::VelocityCommandBoxLimits& box_limits,
	const RDS::VelocityCommandHexagonLimits& hexagon_limits,
	const std::vector<RDS::CollisionPoint>& collision_points,
	float y_coordinate_of_reference_point_for_command_limits,
	float weight_scaling_of_reference_point_for_command_limits,
	float tau,
	float delta,
	float clearance_from_axle_of_final_reference_point)
	: pvcg(box_limits,
		hexagon_limits,
		collision_points,
		y_coordinate_of_reference_point_for_command_limits,
		tau,
		delta)
	, rpg(nominal_command,
		weight_scaling_of_reference_point_for_command_limits,
		clearance_from_axle_of_final_reference_point,
		pvcg)
	, rpvcc(rpg,
		pvcg)
	, rpvo(nominal_command,
		hexagon_limits,
		rpvcc,
		rpg)
{ }
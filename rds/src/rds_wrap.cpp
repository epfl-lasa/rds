#include "rds_wrap.hpp"

RDSWrap::RDSWrap(const RDS::VelocityCommand& nominal_command,
	const RDS::VelocityCommandBoxLimits& box_limits,
	const RDS::VelocityCommandHexagonLimits& hexagon_limits,
	const std::vector<RDS::CollisionPoint>& collision_points,
	float y_coordinate_of_reference_point_for_command_limits,
	float weight_scaling_of_reference_point_for_command_limits,
	float tau,
	float delta,
	float clearance_from_axle_of_final_reference_point,
	bool unilateral_velocity_shift,
	float y_coordinate_of_reference_biasing_point,
	float weight_of_reference_biasing_point,
	bool use_exponential_weighting,
	bool holonomic)
	: pvcg(box_limits,
		hexagon_limits,
		collision_points,
		y_coordinate_of_reference_point_for_command_limits,
		tau,
		delta,
		unilateral_velocity_shift,
		holonomic)
	, rpg(nominal_command,
		weight_scaling_of_reference_point_for_command_limits,
		clearance_from_axle_of_final_reference_point,
		y_coordinate_of_reference_biasing_point,
		weight_of_reference_biasing_point,
		pvcg,
		use_exponential_weighting,
		holonomic)
	, rpvcc(rpg,
		pvcg,
		holonomic)
	, rpvo(nominal_command,
		hexagon_limits,
		rpvcc,
		rpg,
		holonomic)
{ }
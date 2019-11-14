#include "rds_wrap.hpp"
#include "rds_core.hpp"

RDS::VelocityCommand compute_RDS_command(const RDS::VelocityCommand& nominal_command,
	const RDS::VelocityCommandBoxLimits& box_limits,
	const RDS::VelocityCommandHexagonLimits& hexagon_limits,
	const std::vector<RDS::CollisionPoint>& collision_points,
	float y_coordinate_of_reference_point_for_command_limits,
	float weight_scaling_of_reference_point_for_command_limits,
	float tau,
	float delta,
	float clearance_from_axle_of_final_reference_point)
{
	// use rds-core functions
	return RDS::VelocityCommand(0.f, 0.f);
}
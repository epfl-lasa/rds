#include "rds_wrap.hpp"
#include "rds_core.hpp"

/*
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
	RDSWrap rds_wrap(nominal_command,
		box_limits,
		hexagon_limits,
		collision_points,
		y_coordinate_of_reference_point_for_command_limits,
		weight_scaling_of_reference_point_for_command_limits,
		tau,
		delta,
		clearance_from_axle_of_final_reference_point);

	return rds_wrap.getCommandSolution();
}
*/


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
#ifndef RDS_WRAP_HPP
#define RDS_WRAP_HPP

#include "rds_core.hpp"
#include "differential_drive_kinematics.hpp"
#include "collision_point.hpp"
#include "geometry.hpp"

#include <vector>

/*
RDS::VelocityCommand compute_RDS_command(const RDS::VelocityCommand& nominal_command,
	const RDS::VelocityCommandBoxLimits& box_limits,
	const RDS::VelocityCommandHexagonLimits& hexagon_limits,
	const std::vector<RDS::CollisionPoint>& collision_points,
	float y_coordinate_of_reference_point_for_command_limits = 0.5f,
	float weight_scaling_of_reference_point_for_command_limits = 1.f,
	float tau = 2.f,
	float delta = 0.1f,
	float clearance_from_axle_of_final_reference_point = 0.15f);*/

struct RDSWrap
{
	RDSWrap(const RDS::VelocityCommand& nominal_command,
		const RDS::VelocityCommandBoxLimits& box_limits,
		const RDS::VelocityCommandHexagonLimits& hexagon_limits,
		const std::vector<RDS::CollisionPoint>& collision_points,
		float y_coordinate_of_reference_point_for_command_limits = 0.5f,
		float weight_scaling_of_reference_point_for_command_limits = 1.f,
		float tau = 2.f,
		float delta = 0.1f,
		float clearance_from_axle_of_final_reference_point = 0.15f);

	const RDS::VelocityCommand& getCommandSolution() const
	{
		return rpvo.getCommandSolution();
	}

	const Geometry2D::Vec2& getReferencePoint() const
	{
		return rpg.getReferencePoint();
	}

	const Geometry2D::Vec2& getReferencePointVelocitySolution() const
	{
		return rpvo.getReferencePointVelocitySolution();
	}

	const std::vector<Geometry2D::HalfPlane2>& getReferencePointVelocityConstraints() const
	{
		return rpvcc.getConstraints();
	}

	const Geometry2D::Vec2& getScaledShiftedSolution() const
	{
		return rpvo.getScaledShiftedSolution();
	}

	const std::vector<Geometry2D::HalfPlane2>& getScaledShiftedConstraints() const
	{
		return rpvo.getScaledShiftedConstraints();
	}

	const std::vector<RDS::PointVelocityConstraint>& getPointVelocityCollisionConstraints() const
	{
		return pvcg.getCollisionConstraints();
	}

private:
	RDS::PointVelocityConstraintGenerator pvcg;
	RDS::ReferencePointGenerator rpg;
	RDS::ReferencePointVelocityConstraintCompiler rpvcc;
	RDS::ReferencePointVelocityOptimization rpvo;
};

#endif
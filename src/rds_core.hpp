#ifndef RDS_CORE_HPP
#define RDS_CORE_HPP

#include "differential_drive_kinematics.hpp"
#include "collision_point.hpp"
#include "geometry.hpp"

#include <vector>

namespace RDS
{
	struct PointVelocityConstraintGenerator
	{
		PointVelocityConstraintGenerator(const VelocityCommandBoxLimits& box_limits,
			const VelocityCommandHexagonLimits& hexagon_limits,
			const std::vector<CollisionPoint>& collision_points,
			float y_coordinate_of_reference_point_for_command_limits,
			float tau,
			float delta);

		const std::vector<PointVelocityConstraint>& getLimitConstraints() const
		{
			return limit_constraints;
		}

		const std::vector<PointVelocityConstraint>& getCollisionConstraints() const
		{
			return collision_constraints;
		}

	private:
		std::vector<PointVelocityConstraint> limit_constraints;
		std::vector<PointVelocityConstraint> collision_constraints;
	};

	struct ReferencePointGenerator
	{
		ReferencePointGenerator(const VelocityCommand& nominal_command,
			float weight_scaling_of_reference_point_for_command_limits,
			float clearance_from_axle_of_final_reference_point,
			const PointVelocityConstraintGenerator& pvcg);

		const Geometry2D::Vec2& getReferencePoint() const
		{
			return reference_point;
		}

	private:
		Geometry2D::Vec2 reference_point;
	};

	struct ReferencePointVelocityConstraintCompiler
	{
		ReferencePointVelocityConstraintCompiler(const ReferencePointGenerator& rpg,
			const PointVelocityConstraintGenerator& pvcg);

		const std::vector<Geometry2D::HalfPlane2>& getConstraints() const
		{
			return constraints;
		}

	private:
		std::vector<Geometry2D::HalfPlane2> constraints;
	};
}

#endif
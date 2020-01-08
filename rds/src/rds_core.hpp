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
			float delta,
			bool unilateral_velocity_shift);

		const std::vector<PointVelocityConstraint>& getBoxLimitConstraints() const
		{
			return box_limit_constraints;
		}

		const std::vector<PointVelocityConstraint>& getHexagonLimitConstraints() const
		{
			return hexagon_limit_constraints;
		}

		const std::vector<PointVelocityConstraint>& getCollisionConstraints() const
		{
			return collision_constraints;
		}

	private:
		std::vector<PointVelocityConstraint> box_limit_constraints;
		std::vector<PointVelocityConstraint> hexagon_limit_constraints;
		std::vector<PointVelocityConstraint> collision_constraints;
	};

	struct ReferencePointGenerator
	{
		ReferencePointGenerator(const VelocityCommand& nominal_command,
			float weight_scaling_of_reference_point_for_command_limits,
			float clearance_from_axle_of_final_reference_point,
			float y_coordinate_of_reference_biasing_point,
			float weight_of_reference_biasing_point,
			const PointVelocityConstraintGenerator& pvcg,
			bool use_exponential_weighting = false);

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

	struct ReferencePointVelocityOptimization
	{
		ReferencePointVelocityOptimization(const VelocityCommand& nominal_command,
			const VelocityCommandHexagonLimits& hexagon_limits,
			const ReferencePointVelocityConstraintCompiler& rpvcc,
			const ReferencePointGenerator& rpg);

		const VelocityCommand& getCommandSolution() const
		{
			return command_solution;
		}

		const Geometry2D::Vec2& getReferencePointVelocitySolution() const
		{
			return reference_point_velocity_solution;
		}

		const Geometry2D::Vec2& getScaledShiftedSolution() const
		{
			return scaled_shifted_solution;
		}

		const std::vector<Geometry2D::HalfPlane2>& getScaledShiftedConstraints() const
		{
			return scaled_shifted_constraints;
		}

		bool isFeasible() const
		{
			return feasible;
		}

		class HexagonLimitsException { };

	private:
		void computeAndSetShiftAndScaling(const VelocityCommand& nominal_command,
			const VelocityCommandHexagonLimits& hexagon_limits,
			const ReferencePointGenerator& rpg);
		
		Geometry2D::Vec2 shift;
		float scaling;

		bool feasible;
		VelocityCommand command_solution;
		Geometry2D::Vec2 reference_point_velocity_solution;
		Geometry2D::Vec2 scaled_shifted_solution;
		std::vector<Geometry2D::HalfPlane2> scaled_shifted_constraints;
	};
}

#endif
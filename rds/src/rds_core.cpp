#include "rds_core.hpp"
#include "distance_minimizer.hpp"

#include <cmath>
#include <iostream>

namespace RDS
{
	PointVelocityConstraintGenerator::PointVelocityConstraintGenerator(const VelocityCommandBoxLimits& box_limits,
			const VelocityCommandHexagonLimits& hexagon_limits,
			const std::vector<CollisionPoint>& collision_points,
			float y_coordinate_of_reference_point_for_command_limits,
			float tau,
			float delta)
	{
		Geometry2D::Vec2 p_ref(0.f, y_coordinate_of_reference_point_for_command_limits);

		// transform box_limits to velocity constraints for the reference point p_ref
		VelocityCommand box_center(0.5f*(box_limits.min_linear + box_limits.max_linear), 
			0.5f*(box_limits.min_angular + box_limits.max_angular));
		VelocityCommand min_lin_min_ang_box(box_limits.min_linear, box_limits.min_angular);
		VelocityCommand min_lin_max_ang_box(box_limits.min_linear, box_limits.max_angular);
		VelocityCommand max_lin_min_ang_box(box_limits.max_linear, box_limits.min_angular);
		VelocityCommand max_lin_max_ang_box(box_limits.max_linear, box_limits.max_angular);

		box_limit_constraints.push_back(PointVelocityConstraint(p_ref, Geometry2D::HalfPlane2(box_center.pointVelocity(p_ref),
			min_lin_min_ang_box.pointVelocity(p_ref), min_lin_max_ang_box.pointVelocity(p_ref))));
		box_limit_constraints.push_back(PointVelocityConstraint(p_ref, Geometry2D::HalfPlane2(box_center.pointVelocity(p_ref),
			min_lin_min_ang_box.pointVelocity(p_ref), max_lin_min_ang_box.pointVelocity(p_ref))));
		box_limit_constraints.push_back(PointVelocityConstraint(p_ref, Geometry2D::HalfPlane2(box_center.pointVelocity(p_ref),
			max_lin_max_ang_box.pointVelocity(p_ref), min_lin_max_ang_box.pointVelocity(p_ref))));
		box_limit_constraints.push_back(PointVelocityConstraint(p_ref, Geometry2D::HalfPlane2(box_center.pointVelocity(p_ref),
			max_lin_max_ang_box.pointVelocity(p_ref), max_lin_min_ang_box.pointVelocity(p_ref))));

		// transform hexagon_limits to velocity constraints for the reference point p_ref
		VelocityCommand min_lin_min_ang_hex(hexagon_limits.min_linear, -hexagon_limits.absolute_angular_at_min_linear);
		VelocityCommand min_lin_max_ang_hex(hexagon_limits.min_linear, hexagon_limits.absolute_angular_at_min_linear);
		VelocityCommand zero_lin_min_ang_hex(0.f, -hexagon_limits.absolute_angular_at_zero_linear);
		VelocityCommand zero_lin_max_ang_hex(0.f, hexagon_limits.absolute_angular_at_zero_linear);
		VelocityCommand max_lin_min_ang_hex(hexagon_limits.max_linear, -hexagon_limits.absolute_angular_at_max_linear);
		VelocityCommand max_lin_max_ang_hex(hexagon_limits.max_linear, hexagon_limits.absolute_angular_at_max_linear);

		hexagon_limit_constraints.push_back(PointVelocityConstraint(p_ref, Geometry2D::HalfPlane2(Geometry2D::Vec2(0.f, 0.f),
			min_lin_min_ang_hex.pointVelocity(p_ref), zero_lin_min_ang_hex.pointVelocity(p_ref))));
		hexagon_limit_constraints.push_back(PointVelocityConstraint(p_ref, Geometry2D::HalfPlane2(Geometry2D::Vec2(0.f, 0.f),
			zero_lin_min_ang_hex.pointVelocity(p_ref), max_lin_min_ang_hex.pointVelocity(p_ref))));
		hexagon_limit_constraints.push_back(PointVelocityConstraint(p_ref, Geometry2D::HalfPlane2(Geometry2D::Vec2(0.f, 0.f),
			min_lin_max_ang_hex.pointVelocity(p_ref), zero_lin_max_ang_hex.pointVelocity(p_ref))));
		hexagon_limit_constraints.push_back(PointVelocityConstraint(p_ref, Geometry2D::HalfPlane2(Geometry2D::Vec2(0.f, 0.f),
			zero_lin_max_ang_hex.pointVelocity(p_ref), max_lin_max_ang_hex.pointVelocity(p_ref))));
		if (0.f < hexagon_limits.absolute_angular_at_min_linear)
		{
			hexagon_limit_constraints.push_back(PointVelocityConstraint(p_ref, Geometry2D::HalfPlane2(Geometry2D::Vec2(0.f, 0.f),
				min_lin_min_ang_hex.pointVelocity(p_ref), min_lin_max_ang_hex.pointVelocity(p_ref))));
		}
		if (0.f < hexagon_limits.absolute_angular_at_max_linear)
		{
			hexagon_limit_constraints.push_back(PointVelocityConstraint(p_ref, Geometry2D::HalfPlane2(Geometry2D::Vec2(0.f, 0.f),
				max_lin_min_ang_hex.pointVelocity(p_ref), max_lin_max_ang_hex.pointVelocity(p_ref))));
		}

		// generate constraints from the collision points
		for (auto& cp : collision_points)
			collision_constraints.push_back(PointVelocityConstraint(cp.p, cp.createPointPVelocityConstraint(tau, delta)));
	}

	ReferencePointGenerator::ReferencePointGenerator(const VelocityCommand& nominal_command,
		float weight_scaling_of_reference_point_for_command_limits,
		float clearance_from_axle_of_final_reference_point,
		const PointVelocityConstraintGenerator& pvcg)
	{
		reference_point = Geometry2D::Vec2(0.f, 0.f);
		float unnormalized_weight_sum = 0.f;

		for (auto& pvc : pvcg.getBoxLimitConstraints())
		{
			float weight = pvc.h.signedDistance(nominal_command.pointVelocity(pvc.p));
			if (weight > 0.f)
			{
				weight *= weight_scaling_of_reference_point_for_command_limits;
				reference_point = reference_point + weight*pvc.p;
				unnormalized_weight_sum += weight;
			}
		}
		for (auto& pvc : pvcg.getHexagonLimitConstraints())
		{
			float weight = pvc.h.signedDistance(nominal_command.pointVelocity(pvc.p));
			if (weight > 0.f)
			{
				weight *= weight_scaling_of_reference_point_for_command_limits;
				reference_point = reference_point + weight*pvc.p;
				unnormalized_weight_sum += weight;
			}
		}
		for (auto& pvc : pvcg.getCollisionConstraints()) // not scaling weights here
		{
			float weight = pvc.h.signedDistance(nominal_command.pointVelocity(pvc.p));
			if (weight > 0.f)
			{
				reference_point = reference_point + weight*pvc.p;
				unnormalized_weight_sum += weight;
			}
		}
		reference_point = reference_point/(unnormalized_weight_sum + 0.0001f);
		if (reference_point.y < 0.f)
			reference_point.y = -reference_point.y;
		if (reference_point.y < clearance_from_axle_of_final_reference_point)
			reference_point.y = clearance_from_axle_of_final_reference_point;
	}

	ReferencePointVelocityConstraintCompiler::ReferencePointVelocityConstraintCompiler(const ReferencePointGenerator& rpg,
			const PointVelocityConstraintGenerator& pvcg)
	{
		for (auto& pvc : pvcg.getBoxLimitConstraints())
			constraints.push_back(transformPointAVelocityConstraintToPointB(pvc, rpg.getReferencePoint()));
		for (auto& pvc : pvcg.getHexagonLimitConstraints())
			constraints.push_back(transformPointAVelocityConstraintToPointB(pvc, rpg.getReferencePoint()));
		for (auto& pvc : pvcg.getCollisionConstraints())
			constraints.push_back(transformPointAVelocityConstraintToPointB(pvc, rpg.getReferencePoint()));
	}

	ReferencePointVelocityOptimization::ReferencePointVelocityOptimization(const VelocityCommand& nominal_command,
		const VelocityCommandHexagonLimits& hexagon_limits,
		const ReferencePointVelocityConstraintCompiler& rpvcc,
		const ReferencePointGenerator& rpg)
	{
		computeAndSetShiftAndScaling(nominal_command, hexagon_limits, rpg);

		scaled_shifted_constraints = rpvcc.getConstraints();
		for (auto& h : scaled_shifted_constraints)
			h.shift(shift).rescale(scaling);

		try {
			scaled_shifted_solution = Geometry2D::DistanceMinimizer::IncrementalDistanceMinimization(scaled_shifted_constraints);
			feasible = true;
		}
		catch (Geometry2D::DistanceMinimizer::InfeasibilityException e) {
			//std::cout << "Infeasible constraints ..." << std::endl;
			feasible = false;
		}

		reference_point_velocity_solution = scaled_shifted_solution/scaling - shift;

		command_solution = VelocityCommand(rpg.getReferencePoint(), reference_point_velocity_solution);
	}

	void ReferencePointVelocityOptimization::computeAndSetShiftAndScaling(const VelocityCommand& nominal_command,
		const VelocityCommandHexagonLimits& hexagon_limits,
		const ReferencePointGenerator& rpg)
	{
		Geometry2D::Vec2 p_ref = rpg.getReferencePoint();
		shift = -1.f*nominal_command.pointVelocity(p_ref);
		
		VelocityCommand min_lin_min_ang_hex(hexagon_limits.min_linear, -hexagon_limits.absolute_angular_at_min_linear);
		VelocityCommand min_lin_max_ang_hex(hexagon_limits.min_linear, hexagon_limits.absolute_angular_at_min_linear);
		VelocityCommand zero_lin_min_ang_hex(0.f, -hexagon_limits.absolute_angular_at_zero_linear);
		VelocityCommand zero_lin_max_ang_hex(0.f, hexagon_limits.absolute_angular_at_zero_linear);
		VelocityCommand max_lin_min_ang_hex(hexagon_limits.max_linear, -hexagon_limits.absolute_angular_at_max_linear);
		VelocityCommand max_lin_max_ang_hex(hexagon_limits.max_linear, hexagon_limits.absolute_angular_at_max_linear);

		Geometry2D::Vec2 v_p_ref_limits_corners[] =
		{
			min_lin_min_ang_hex.pointVelocity(p_ref),
			min_lin_max_ang_hex.pointVelocity(p_ref),
			zero_lin_min_ang_hex.pointVelocity(p_ref),
			zero_lin_max_ang_hex.pointVelocity(p_ref),
			max_lin_min_ang_hex.pointVelocity(p_ref),
			max_lin_max_ang_hex.pointVelocity(p_ref)
		};
		float max_abs_coordinate = 0.f;
		for (int i = 0; i < 6; i++)
		{
			if (std::abs((v_p_ref_limits_corners[i] + shift).x) > max_abs_coordinate)
				max_abs_coordinate = std::abs((v_p_ref_limits_corners[i] + shift).x);
			if (std::abs((v_p_ref_limits_corners[i] + shift).y) > max_abs_coordinate)
				max_abs_coordinate = std::abs((v_p_ref_limits_corners[i] + shift).y);
		}
		if (max_abs_coordinate == 0.f)
			throw HexagonLimitsException();
		scaling = 0.5f/(max_abs_coordinate + 0.01f);
	}
}

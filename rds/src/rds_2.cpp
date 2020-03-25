#include "rds_2.hpp"
#include "distance_minimizer.hpp"
#include <cmath>
#include <iostream>

namespace Geometry2D
{
	RDS2::RDS2(float T, float D, float delta, float v_max)
	: T(T), D(D), delta(delta), v_max(v_max), v_max_sqrt_2(v_max*std::sqrt(2.f))
	{ }

	void RDS2::computeCorrectedVelocity(const Capsule& robot_shape, const Vec2& p_ref, const Vec2& v_nominal_p_ref,
		const std::vector<AdditionalPrimitives2D::Circle>& objects, Vec2* v_corrected_p_ref)
	{
		std::vector<HalfPlane2> constraints_tmp;
		generateConstraints(robot_shape, p_ref, objects, &constraints_tmp);
		solve(v_nominal_p_ref, constraints_tmp, v_corrected_p_ref);
	}

	void RDS2::computeCorrectedVelocity(const AdditionalPrimitives2D::Circle& robot_shape, const Vec2& p_ref,
		const Vec2& v_nominal_p_ref, const std::vector<AdditionalPrimitives2D::Circle>& circle_objects,
		const std::vector<Capsule>& capsule_objects, Vec2* v_corrected_p_ref)
	{
		std::vector<HalfPlane2> constraints_tmp;
		generateConstraints(robot_shape, p_ref, circle_objects, capsule_objects, &constraints_tmp);
		solve(v_nominal_p_ref, constraints_tmp, v_corrected_p_ref);
	}

	void RDS2::generateConstraints(const AdditionalPrimitives2D::Circle& robot_shape, const Vec2& p_ref,
		const std::vector<AdditionalPrimitives2D::Circle>& circle_objects,
		const std::vector<Capsule>& capsule_objects, std::vector<HalfPlane2>* constraints)
	{
		constraints->resize(0);
		for (auto& cir_obj : circle_objects)
		{
			generateAndAddConstraint(robot_shape.center, robot_shape.radius, cir_obj.center,
				cir_obj.radius, p_ref, constraints);
		}
		for (auto& cap_obj : capsule_objects)
		{
			Vec2 pt_segment;
			cap_obj.closestMidLineSegmentPoint(robot_shape.center, &pt_segment);
			generateAndAddConstraint(robot_shape.center, robot_shape.radius, pt_segment, cap_obj.radius(),
				p_ref, constraints);
		}
	}

	void RDS2::generateConstraints(const Capsule& robot_shape, const Vec2& p_ref,
		const std::vector<AdditionalPrimitives2D::Circle>& objects, std::vector<HalfPlane2>* constraints)
	{
		constraints->resize(0);
		for (std::vector<HalfPlane2>::size_type i = 0; i != objects.size(); i++)
		{
			Vec2 pt_segment;
			robot_shape.closestMidLineSegmentPoint(objects[i].center, &pt_segment);
			generateAndAddConstraint(pt_segment, robot_shape.radius(), objects[i].center, objects[i].radius,
				p_ref, constraints);
		}
	}

	void RDS2::generateAndAddConstraint(const Geometry2D::Vec2& robot_point, float robot_radius,
		const Geometry2D::Vec2& object_point, float object_radius, const Vec2& p_ref, std::vector<HalfPlane2>* constraints)
	{
		float centers_distance = (object_point - robot_point).norm();
		float arg_sqrt = (centers_distance - object_radius - robot_radius)/D - delta;
		float b = 0.f;
		if (arg_sqrt > 0.f)
			b = D/T*std::sqrt(arg_sqrt);

		Vec2 n_centers = (object_point - robot_point).normalized();
		Vec2 n_constraint_tmp(n_centers.x*robot_point.y/p_ref.y + n_centers.y*(p_ref.x -
			robot_point.x)/p_ref.y, n_centers.y);

		if ((v_max_sqrt_2 + 0.01f)*n_constraint_tmp.norm() > b)
			constraints->push_back(HalfPlane2(n_constraint_tmp, b/n_constraint_tmp.norm()));
	}

	void RDS2::solve(const Vec2& v_nominal, std::vector<HalfPlane2>& constraints_tmp, Vec2* v_corrected)
	{
		// shift and scale constraints
		constraints_tmp.push_back(HalfPlane2(Vec2(1.f, 0.f), v_max));
		constraints_tmp.push_back(HalfPlane2(Vec2(-1.f, 0.f), v_max));
		constraints_tmp.push_back(HalfPlane2(Vec2(0.f, 1.f), v_max));
		constraints_tmp.push_back(HalfPlane2(Vec2(0.f, -1.f), v_max));

		constraints = constraints_tmp;

		Vec2 shift = -1.f*v_nominal;
		float scaling = 0.5f/(shift.norm() + v_max + 0.01f);
		for (auto& c : constraints_tmp)
		{
			c.shift(shift);
			c.rescale(scaling);
		}

		// solve the normalized problem
		try
		{
			Vec2 scaled_shifted_solution = DistanceMinimizer::IncrementalDistanceMinimization(constraints_tmp);
			*v_corrected = scaled_shifted_solution/scaling - shift;
		}
		catch (Geometry2D::DistanceMinimizer::InfeasibilityException e)
		{
			*v_corrected = Vec2(0.f, 0.f);
			std::cout << "Infeasible constraints" << std::endl;
		}
	}
}
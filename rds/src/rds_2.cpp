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

	void RDS2::generateConstraints(const Capsule& robot_shape, const Vec2& p_ref,
		const std::vector<AdditionalPrimitives2D::Circle>& objects,
		std::vector<HalfPlane2>* constraints)
	{
		constraints->resize(0);
		for (std::vector<HalfPlane2>::size_type i = 0; i != objects.size(); i++)
		{
			Vec2 pt_segment;
			robot_shape.closestMidLineSegmentPoint(objects[i].center, &pt_segment);
			float centers_distance = (objects[i].center - pt_segment).norm();
			float arg_sqrt = (centers_distance - objects[i].radius - robot_shape.radius)/D - delta;
			float b = 0.f;
			if (arg_sqrt > 0.f)
				b = D/T*std::sqrt(arg_sqrt);

			Vec2 n_centers = (objects[i].center - pt_segment).normalized();
			Vec2 n_constraint_tmp(n_centers.x*pt_segment.y/p_ref.y + n_centers.y*(p_ref.x -
				pt_segment.x)/p_ref.y, n_centers.y);

			if ((v_max_sqrt_2 + 0.01f)*n_constraint_tmp.norm() > b)
				constraints->push_back(HalfPlane2(n_constraint_tmp, b/n_constraint_tmp.norm()));
		}
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
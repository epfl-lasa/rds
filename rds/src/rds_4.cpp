#include "rds_4.hpp"
#include "RVO.hpp"
#include "distance_minimizer.hpp"
#include <cmath>
#include <iostream>

namespace Geometry2D
{
	RDS4::RDS4(float tau, float delta, float v_max)
	: tau(tau), delta(delta), v_max(v_max), v_max_sqrt_2(v_max*std::sqrt(2.f))
	{ }

	void RDS4::computeCorrectedVelocity(const Capsule& robot_shape, const Vec2& p_ref, const Vec2& v_nominal_p_ref,
		const std::vector<MovingCircle>& objects, Vec2* v_corrected_p_ref)
	{
		std::vector<HalfPlane2> constraints_tmp;
		generateConstraints(robot_shape, p_ref, v_nominal_p_ref, objects, &constraints_tmp);
		solve(v_nominal_p_ref, constraints_tmp, v_corrected_p_ref);
	}

	void RDS4::generateConstraints(const Capsule& robot_shape, const Vec2& p_ref, const Vec2& v_nominal_p_ref,
		const std::vector<MovingCircle>& objects, std::vector<HalfPlane2>* constraints)
	{
		constraints->resize(0);
		for (std::vector<HalfPlane2>::size_type i = 0; i != objects.size(); i++)
		{
			Vec2 pt_segment;
			robot_shape.closestMidLineSegmentPoint(objects[i].circle.center, &pt_segment);
			generateAndAddConstraint(pt_segment, robot_shape.radius, objects[i].circle.center,
				objects[i].circle.radius, objects[i].velocity, p_ref, v_nominal_p_ref, constraints);
		}
	}

	void RDS4::generateAndAddConstraint(const Vec2& robot_point, float robot_radius,
		const Vec2& object_point, float object_radius, const Vec2& object_velocity,
		const Vec2& p_ref, const Vec2& v_nominal_p_ref, std::vector<HalfPlane2>* constraints)
	{
		float radius_sum = robot_radius + object_radius + delta;
		if ((robot_point.x == 0.f) && (p_ref.x == 0.f))
		{
			float sigma = std::max(1.f, std::abs(robot_point.y/p_ref.y));
			float v_max_sqrt_2_robot_point = sigma*v_max_sqrt_2;
			if (((robot_point - object_point).norm() - radius_sum)/tau > v_max_sqrt_2_robot_point + 0.01f)
				return;
		}
		RVO crvo_computer(tau, delta);
		Vec2 relative_position = object_point - robot_point;
		Vec2 relative_velocity_preferred = Vec2(v_nominal_p_ref.x*robot_point.y/p_ref.y,
			v_nominal_p_ref.x*(p_ref.x - robot_point.x)/p_ref.y + v_nominal_p_ref.y) - object_velocity;
		HalfPlane2 crvo;
		crvo_computer.computeConvexRVO(relative_position, relative_velocity_preferred, radius_sum, &crvo);

		crvo.shift(object_velocity);
		if (crvo.getOffset() < 0.f)
			crvo.shift(crvo.getNormal()*(-crvo.getOffset()));

		const Vec2& n(crvo.getNormal());
		Vec2 n_constraint_tmp(n.x*robot_point.y/p_ref.y + n.y*(p_ref.x - robot_point.x)/p_ref.y, n.y);
		float b = crvo.getOffset();
		if ((v_max_sqrt_2 + 0.01f)*n_constraint_tmp.norm() > b)
			constraints->push_back(HalfPlane2(n_constraint_tmp, b/n_constraint_tmp.norm()));
	}
	
	void RDS4::solve(const Vec2& v_nominal, std::vector<HalfPlane2>& constraints_tmp, Vec2* v_corrected)
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
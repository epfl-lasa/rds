#include "rds_5.hpp"
#include "RVO.hpp"
#include "distance_minimizer.hpp"
#include <cmath>
#include <iostream>

namespace Geometry2D
{
	RDS5::RDS5(float tau, float delta, float y_p_ref, const VWBox& vw_box_limits,
		const VWDiamond& vw_diamond_limits)
	: tau(tau), delta(delta), y_p_ref(y_p_ref)
	{
		// map vw-limits to cartesian velocity constraints for the reference point
		// for box-limits
		v_box_x_min = -vw_box_limits.w_max*y_p_ref;
		v_box_x_max = -vw_box_limits.w_min*y_p_ref;
		v_box_y_min = vw_box_limits.v_min;
		v_box_y_max = vw_box_limits.v_max;
		constraints_box_limits.push_back(HalfPlane2(Vec2(-1.f, 0.f), -v_box_x_min));
		constraints_box_limits.push_back(HalfPlane2(Vec2(1.f, 0.f), v_box_x_max));
		constraints_box_limits.push_back(HalfPlane2(Vec2(0.f, -1.f), -v_box_y_min));
		constraints_box_limits.push_back(HalfPlane2(Vec2(0.f, 1.f), v_box_y_max));
		// for diamond limits
		Vec2 corner_left(-vw_diamond_limits.w_abs_max*y_p_ref, vw_diamond_limits.v_at_w_abs_max);
		Vec2 corner_lower(0.f, vw_diamond_limits.v_min);
		Vec2 corner_right(vw_diamond_limits.w_abs_max*y_p_ref, vw_diamond_limits.v_at_w_abs_max);
		Vec2 corner_upper(0.f, vw_diamond_limits.v_max);
		std::vector<Vec2> diamond_corners = {corner_left, corner_lower, corner_right, corner_upper, corner_left};
		for (int i = 0; i < 4; i++)
		{
			constraints_diamond_limits.push_back(HalfPlane2(Vec2(0.f, 0.f), //p_interior
				diamond_corners[i], diamond_corners[i + 1])); // 2 p_boundary
		}
		v_p_ref_radial_max = 0.f;
		for (auto& v_corner : diamond_corners)
		{
			if (v_corner.norm() > v_p_ref_radial_max)
				v_p_ref_radial_max = v_corner.norm();
		}
	}

	void RDS5::computeCorrectedVelocity(const Capsule& robot_shape, const Vec2& v_nominal_p_ref,
		const std::vector<MovingCircle>& objects, Vec2* v_corrected_p_ref)
	{
		Vec2 p_ref(0.f, y_p_ref);
		std::vector<HalfPlane2> constraints_tmp;
		generateConstraints(robot_shape, p_ref, v_nominal_p_ref, objects, &constraints_tmp);
		solve(v_nominal_p_ref, constraints_tmp, v_corrected_p_ref);
		// crop to satisfy the box limits and save their halfplanes (to show that they exist)
		v_corrected_p_ref->x = std::min(v_box_x_max, std::max(v_box_x_min, v_corrected_p_ref->x));
		v_corrected_p_ref->y = std::min(v_box_y_max, std::max(v_box_y_min, v_corrected_p_ref->y));
		for (auto& h : constraints_box_limits)
			constraints.push_back(h);
	}

	void RDS5::generateConstraints(const Capsule& robot_shape, const Vec2& p_ref, const Vec2& v_nominal_p_ref,
		const std::vector<MovingCircle>& objects, std::vector<HalfPlane2>* constraints)
	{
		constraints->resize(0);
		for (std::vector<HalfPlane2>::size_type i = 0; i != objects.size(); i++)
		{
			Vec2 pt_segment;
			robot_shape.closestMidLineSegmentPoint(objects[i].circle.center, &pt_segment);
			generateAndAddConstraint(pt_segment, robot_shape.radius(), objects[i].circle.center,
				objects[i].circle.radius, objects[i].velocity, p_ref, v_nominal_p_ref, constraints);
		}
	}

	void RDS5::generateAndAddConstraint(const Vec2& robot_point, float robot_radius,
		const Vec2& object_point, float object_radius, const Vec2& object_velocity,
		const Vec2& p_ref, const Vec2& v_nominal_p_ref, std::vector<HalfPlane2>* constraints)
	{
		float radius_sum = robot_radius + object_radius + delta;
		if ((robot_point.x == 0.f) && (p_ref.x == 0.f))
		{
			float sigma = std::max(1.f, std::abs(robot_point.y/p_ref.y));
			float v_robot_point_radial_max = sigma*v_p_ref_radial_max;
			if (((robot_point - object_point).norm() - radius_sum)/tau > v_robot_point_radial_max + 0.01f)
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
		if ((v_p_ref_radial_max + 0.01f)*n_constraint_tmp.norm() > b)
			constraints->push_back(HalfPlane2(n_constraint_tmp, b/n_constraint_tmp.norm()));
		float normal_limit = 1.f*robot_radius/tau/v_p_ref_radial_max;
		if (std::abs(n_constraint_tmp.x) < normal_limit)
		{
			float shift_abs = normal_limit - std::abs(n_constraint_tmp.x);
			Vec2 n_constraint_tmp_plus(n.x*(robot_point.y + shift_abs)/
				p_ref.y + n.y*(p_ref.x - robot_point.x)/p_ref.y, n.y);
			Vec2 n_constraint_tmp_minus(n.x*(robot_point.y - shift_abs)/
				p_ref.y + n.y*(p_ref.x - robot_point.x)/p_ref.y, n.y);
			if ((v_p_ref_radial_max + 0.01f)*n_constraint_tmp_plus.norm() > b)
				constraints->push_back(HalfPlane2(n_constraint_tmp_plus, b/n_constraint_tmp_plus.norm()));
			if ((v_p_ref_radial_max + 0.01f)*n_constraint_tmp_minus.norm() > b)
				constraints->push_back(HalfPlane2(n_constraint_tmp_minus, b/n_constraint_tmp_minus.norm()));
		}
	}
	
	void RDS5::solve(const Vec2& v_nominal, std::vector<HalfPlane2>& constraints_tmp, Vec2* v_corrected)
	{
		// add constraints due to diamond limits
		for (auto& h : constraints_diamond_limits)
			constraints_tmp.push_back(h);
		// save the constraints for visualization
		constraints = constraints_tmp;
		// shift and scale the constraints
		Vec2 shift = -1.f*v_nominal;
		float scaling = 0.5f/(shift.norm() + v_p_ref_radial_max + 0.01f);
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
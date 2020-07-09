#include "rds_5.hpp"
#include "my_rvo.hpp"
#include "distance_minimizer.hpp"
#include <Agent.h> // from RVO2
#include <cmath>
#include <iostream>

namespace Geometry2D
{
	RDS5::RDS5(float tau, float delta, float y_p_ref, const VWBox& vw_box_limits,
		const VWDiamond& vw_diamond_limits)
	: tau(tau), delta(delta), y_p_ref(y_p_ref)
	, use_previous_command_as_basis(true)
	, use_orca_style_crvo(true)
	, use_conservative_shift(true)
	, n_bounding_circles(0)
	, keep_origin_feasible(true)
	, no_VO_shift_at_contact(false)
	, shift_reduction_range(0.25f)
	, ORCA_implementation(false)
	, ORCA_use_p_ref(false)
	, ORCA_solver(false)
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

	void RDS5::computeCorrectedVelocity(const Capsule& robot_shape,
			const Vec2& v_nominal_p_ref, const Vec2& v_previous_command,
			const std::vector<MovingCircle>& static_objects,
			const std::vector<MovingCircle>& moving_objects,
			Vec2* v_corrected_p_ref)
	{
		if (!ORCA_implementation)
		{
			if (n_bounding_circles > 2)
			{
				bounding_circles = BoundingCircles(n_bounding_circles - 3);
				bounding_circles.fit(robot_shape, delta);
				for (const auto& c : bounding_circles.circles())
				{
					if (std::abs(c.center.y) < 0.05f)
						throw "A bounding circle's center is to close to the wheel axle";
				}
			}
			Vec2 p_ref(0.f, y_p_ref);
			Vec2 basis_command = v_nominal_p_ref;
			if (use_previous_command_as_basis)
				basis_command = v_previous_command;
			std::vector<HalfPlane2> constraints_tmp;
			generateVOConstraints(robot_shape, p_ref, basis_command, moving_objects, &constraints_tmp);
			generateStaticConstraints(robot_shape, p_ref, static_objects, &constraints_tmp);
			solve(v_nominal_p_ref, constraints_tmp, v_corrected_p_ref);
		}
		else
		{
			float y_center = 0.5f*(robot_shape.center_a().y + robot_shape.center_b().y);
			float radius_ORCA = 0.5f*(robot_shape.center_a() - robot_shape.center_b()).norm() + robot_shape.radius();
			if (ORCA_use_p_ref)
			{
				y_center = y_p_ref;
				radius_ORCA = y_center - robot_shape.center_b().y + robot_shape.radius();
			}
			Vec2 v_center_previous_command(v_previous_command);
			v_center_previous_command.x *= y_center/y_p_ref;
			Vec2 v_center_nominal(v_nominal_p_ref);
			v_center_nominal.x *= y_center/y_p_ref;
			std::vector<HalfPlane2> constraints_center_tmp;
			generateConstraintsLikeORCA(radius_ORCA, y_center, v_center_previous_command,
				moving_objects, static_objects, &constraints_center_tmp);
			addLimitConstraints(y_center/y_p_ref, &constraints_center_tmp);
			Vec2 v_center_corrected;
			if (!ORCA_solver)
				solve(v_center_nominal, constraints_center_tmp, &v_center_corrected);
			else
				solveWithORCASolver(v_center_nominal, constraints_center_tmp, &v_center_corrected);
			*v_corrected_p_ref = v_center_corrected;
			v_corrected_p_ref->x /= y_center/y_p_ref;
		}
		// crop to satisfy the box limits and save their halfplanes (to show that they exist)
		if (keep_origin_feasible)
		{
			v_corrected_p_ref->x = std::min(v_box_x_max, std::max(v_box_x_min, v_corrected_p_ref->x));
			v_corrected_p_ref->y = std::min(v_box_y_max, std::max(v_box_y_min, v_corrected_p_ref->y));
			for (auto& h : constraints_box_limits)
				constraints.push_back(h);
		}
	}

	void RDS5::generateVOConstraints(const Capsule& robot_shape, const Vec2& p_ref, const Vec2& v_nominal_p_ref,
		const std::vector<MovingCircle>& objects, std::vector<HalfPlane2>* constraints)
	{
		constraints->resize(0);
		for (std::vector<HalfPlane2>::size_type i = 0; i != objects.size(); i++)
		{
			if (n_bounding_circles > 2)
			{
				for (const auto& c : bounding_circles.circles())
				{
					generateAndAddConstraint(c.center, c.radius, objects[i].circle.center,
						objects[i].circle.radius, objects[i].velocity, p_ref, v_nominal_p_ref, constraints);
				}
			}
			else
			{
				Vec2 pt_segment;
				robot_shape.closestMidLineSegmentPoint(objects[i].circle.center, &pt_segment);
				//pt_segment = p_ref;
				generateAndAddConstraint(pt_segment, robot_shape.radius(), objects[i].circle.center,
					objects[i].circle.radius, objects[i].velocity, p_ref, v_nominal_p_ref, constraints);
			}
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
		MyRVO crvo_computer(tau, delta);
		Vec2 relative_position = object_point - robot_point;
		Vec2 relative_velocity_preferred = Vec2(v_nominal_p_ref.x*robot_point.y/p_ref.y,
			v_nominal_p_ref.x*(p_ref.x - robot_point.x)/p_ref.y + v_nominal_p_ref.y) - object_velocity;
		if (use_conservative_shift)
			relative_velocity_preferred = -1.f*object_velocity;
		HalfPlane2 crvo;
		crvo_computer.computeConvexRVO(relative_position, relative_velocity_preferred, radius_sum, &crvo,
			use_orca_style_crvo);
		crvo.shift(VOShift(object_velocity, relative_position, radius_sum));
		if (crvo.getOffset() < 0.f && keep_origin_feasible)
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
			//if ((v_p_ref_radial_max + 0.01f)*n_constraint_tmp_plus.norm() > b)
			//	constraints->push_back(HalfPlane2(n_constraint_tmp_plus, b/n_constraint_tmp_plus.norm()));
			//if ((v_p_ref_radial_max + 0.01f)*n_constraint_tmp_minus.norm() > b)
			//	constraints->push_back(HalfPlane2(n_constraint_tmp_minus, b/n_constraint_tmp_minus.norm()));
		}
	}

	void RDS5::generateStaticConstraints(const Capsule& robot_shape, const Vec2& p_ref,
		const std::vector<MovingCircle>& static_objects, std::vector<HalfPlane2>* constraints)
	{
		for (const auto& obj : static_objects)
		{
			if (n_bounding_circles > 2)
			{
				for (const auto& c : bounding_circles.circles())
				{
					generateAndAddStaticConstraint(c.center, c.radius, obj.circle.center,
						obj.circle.radius, p_ref, constraints);
				}
			}
			else
			{
				Vec2 robot_point;
				robot_shape.closestMidLineSegmentPoint(obj.circle.center, &robot_point);
				generateAndAddStaticConstraint(robot_point, robot_shape.radius(),
					obj.circle.center, obj.circle.radius, p_ref, constraints);
			}
		}
	}

	void RDS5::generateAndAddStaticConstraint(const Vec2& robot_point,
		float robot_radius, const Vec2& object_point, float object_radius,
		const Vec2& p_ref, std::vector<HalfPlane2>* constraints)
	{
		float radius_sum = robot_radius + object_radius + delta;
		if ((robot_point.x == 0.f) && (p_ref.x == 0.f))
		{
			float sigma = std::max(1.f, std::abs(robot_point.y/p_ref.y));
			float v_robot_point_radial_max = sigma*v_p_ref_radial_max;
			if (((robot_point - object_point).norm() - radius_sum)/tau > v_robot_point_radial_max + 0.01f)
				return;
		}
		Vec2 n = (object_point - robot_point).normalized();
		float b = ((robot_point - object_point).norm() - radius_sum)/tau;
		Vec2 n_constraint_tmp(n.x*robot_point.y/p_ref.y + n.y*(p_ref.x - robot_point.x)/p_ref.y, n.y);
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
			//if ((v_p_ref_radial_max + 0.01f)*n_constraint_tmp_plus.norm() > b)
			//	constraints->push_back(HalfPlane2(n_constraint_tmp_plus, b/n_constraint_tmp_plus.norm()));
			//if ((v_p_ref_radial_max + 0.01f)*n_constraint_tmp_minus.norm() > b)
			//	constraints->push_back(HalfPlane2(n_constraint_tmp_minus, b/n_constraint_tmp_minus.norm()));
		}
	}
	
	void RDS5::solve(const Vec2& v_nominal, std::vector<HalfPlane2>& constraints_tmp, Vec2* v_corrected)
	{
		if (!ORCA_implementation)
		{
			// add constraints due to diamond limits
			for (auto& h : constraints_diamond_limits)
				constraints_tmp.push_back(h);
			if (!keep_origin_feasible)
			{
				for (auto& h : constraints_box_limits)
					constraints_tmp.push_back(h);
			}
		}
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
		if (keep_origin_feasible)
		{
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
		else
		{
			Vec2 scaled_shifted_solution = DistanceMinimizer::IncrementalDistanceMinimization(constraints_tmp);
			*v_corrected = scaled_shifted_solution/scaling - shift;
		}
	}

	void RDS5::generateConstraintsLikeORCA(float radius_ORCA, float y_center,
		const Vec2& basis_command, const std::vector<MovingCircle>& moving_objects,
		const std::vector<MovingCircle>& static_objects, std::vector<HalfPlane2>* constraints)
	{
		for (auto& mc : moving_objects)
		{
			generateAndAddConstraintLikeORCA(y_center, radius_ORCA, mc.circle.center, mc.circle.radius,
				mc.velocity, basis_command, constraints);
		}
		for (auto& mc : static_objects)
		{
			generateAndAddConstraintLikeORCA(y_center, radius_ORCA, mc.circle.center, mc.circle.radius,
				mc.velocity, basis_command, constraints);
		}
	}

	void RDS5::generateAndAddConstraintLikeORCA(float y_robot_center, float robot_radius,
		const Vec2& object_point, float object_radius, const Vec2& object_velocity,
		const Vec2& robot_basis_velocity, std::vector<HalfPlane2>* constraints)
	{
		float radius_sum = robot_radius + object_radius + delta;
		Vec2 robot_point(0.f, y_robot_center);

		float sigma = std::max(1.f, std::abs(y_robot_center/y_p_ref));
		float v_robot_point_radial_max = sigma*v_p_ref_radial_max;
		if (((robot_point - object_point).norm() - radius_sum)/tau > v_robot_point_radial_max + 0.01f)
			return;

		MyRVO crvo_computer(tau, delta);
		Vec2 relative_position = object_point - robot_point;
		Vec2 relative_basis_velocity = robot_basis_velocity - object_velocity;
		if (use_conservative_shift)
			relative_basis_velocity = -1.f*object_velocity;
		HalfPlane2 crvo;
		crvo_computer.computeConvexRVO(relative_position, relative_basis_velocity, radius_sum, &crvo,
			true);
		crvo.shift(VOShift(object_velocity, relative_position, radius_sum));
		if (crvo.getOffset() < 0.f && keep_origin_feasible)
			crvo.shift(crvo.getNormal()*(-crvo.getOffset()));
		constraints->push_back(crvo);
	}

	void RDS5::addLimitConstraints(float y_center_over_y_p_ref, std::vector<HalfPlane2>* constraints)
	{
		HalfPlane2 new_constraint;
		// add constraints due to diamond limits
		for (auto& h : constraints_diamond_limits)
		{
			new_constraint = h;
			transformDiamondConstraint(y_center_over_y_p_ref, &new_constraint);
			constraints->push_back(new_constraint);
		}
		// add constraints due to box limits
		if (!keep_origin_feasible)
		{
			for (auto& h : constraints_box_limits)
			{
				new_constraint = h;
				transformBoxConstraint(y_center_over_y_p_ref, &new_constraint);
				constraints->push_back(new_constraint);
			}
		}
	}

	void RDS5::transformBoxConstraint(float y_new_over_y_old, HalfPlane2* c)
	{
		const Vec2& n(c->getNormal());
		if (std::abs(n.x) > std::abs(n.y))
		{
			float b(c->getOffset());
			if (y_new_over_y_old < 0.f)
				*c = HalfPlane2(-1.f*n, -b*y_new_over_y_old);
			else
				*c = HalfPlane2(n, b*y_new_over_y_old);
		}
	}

	void RDS5::transformDiamondConstraint(float y_new_over_y_old, HalfPlane2* c)
	{
		const Vec2& n(c->getNormal());
		float b(c->getOffset());
		float v_x = b/n.x;
		float v_y = b/n.y;
		float v_x_new = v_x*y_new_over_y_old;
		float v_y_new = v_y;
		*c = HalfPlane2(Vec2(0.f, 0.f), Vec2(v_x_new, 0.f), Vec2(0.f, v_y_new));
	}

	Vec2 RDS5::VOShift(const Vec2& v_obj, const Vec2& relative_position, float combined_radius)
	{
		if (!no_VO_shift_at_contact)
			return v_obj;
		float distance = relative_position.norm() - combined_radius;
		if (distance > shift_reduction_range)
			return v_obj;
		if (distance < 0.f)
			return Vec2(0.f, 0.f);
		float scaling = distance/shift_reduction_range;
		return v_obj*scaling;

	}

	void RDS5::solveWithORCASolver(const Vec2& v_nominal, std::vector<HalfPlane2>& center_constraints, Vec2* v_corrected)
	{
		constraints = center_constraints;
		// infeasible halfplane to the right in the line's direction
		std::vector<RVO::Line> orca_lines;
		RVO::Line line;
		for (auto& h : center_constraints)
		{
			line.point = RVO::Vector2(h.getOrigo().x, h.getOrigo().y);
			line.direction = RVO::Vector2(h.getParallel().x, h.getParallel().y);
			orca_lines.push_back(line);
		}
		RVO::Vector2 preferred_velocity(v_nominal.x, v_nominal.y);
		RVO::Vector2 new_velocity;
		size_t lineFail = RVO::linearProgram2(orca_lines, 10.f, preferred_velocity, false, new_velocity);
		if (lineFail < orca_lines.size())
			RVO::linearProgram3(orca_lines, 0, lineFail, 10.f, new_velocity);
		v_corrected->x = new_velocity.x();
		v_corrected->y = new_velocity.y();
	}
}
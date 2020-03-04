#include "rvo_3_agent.hpp"
#include "RVO.hpp"
#include "distance_minimizer.hpp"
#include <cmath>
#include <iostream>

using Geometry2D::Vec2;
using AdditionalPrimitives2D::Circle;
using Geometry2D::HalfPlane2;
using Geometry2D::Capsule;

RVO3Configuration::RVO3Configuration(float tau, float delta, float v_max, float radius)
	: tau(tau), delta(delta), v_max(v_max), radius(radius), v_max_sqrt_2(v_max*std::sqrt(2))
{ }

void RVO3Agent::stepEuler(float dt, const Vec2& v_nominal,
	const std::vector<MovingCircle>& objects_with_preferred_velocities,
	const MovingCapsule& robot_with_preferred_velocity, unsigned long int skip_index)
{
	std::vector<HalfPlane2> constraints;
	constraints.reserve(objects_with_preferred_velocities.size() + 1 + 4);

	constraints.push_back(HalfPlane2(Vec2(1.f, 0.f), rvo_configuration.v_max));
	constraints.push_back(HalfPlane2(Vec2(-1.f, 0.f), rvo_configuration.v_max));
	constraints.push_back(HalfPlane2(Vec2(0.f, 1.f), rvo_configuration.v_max));
	constraints.push_back(HalfPlane2(Vec2(0.f, -1.f), rvo_configuration.v_max));

	RVO rvo(rvo_configuration.tau, rvo_configuration.delta);
	HalfPlane2 vo_1, vo_2;

	Vec2 pt_segment;
	robot_with_preferred_velocity.capsule.closestMidLineSegmentPoint(position, &pt_segment);
	Vec2 velocity_pt_segment = robot_with_preferred_velocity.velocity_p_ref + Vec2(
		-robot_with_preferred_velocity.omega*(pt_segment - robot_with_preferred_velocity.p_ref).y,
		robot_with_preferred_velocity.omega*(pt_segment - robot_with_preferred_velocity.p_ref).x);
	rvo.computeCoordinativeVelocityObstacles(Circle(position, rvo_configuration.radius),
		Circle(pt_segment, robot_with_preferred_velocity.capsule.radius), v_nominal,
		velocity_pt_segment, &vo_1, &vo_2);
	constraints.push_back(vo_1);

	for (std::vector<MovingCircle>::size_type i = 0; i != objects_with_preferred_velocities.size(); i++)
	{
		if (i == skip_index)
			continue;
		auto& mc = objects_with_preferred_velocities[i];
		float distance = (position - mc.circle.center).norm() - rvo_configuration.radius - mc.circle.radius - rvo_configuration.delta;
		float margin = (distance/rvo_configuration.tau - (v_nominal.norm() + mc.velocity.norm()))/2.f;
		if (v_nominal.norm() + rvo_configuration.v_max_sqrt_2 + 0.01f < margin)
			continue;
		rvo.computeCoordinativeVelocityObstacles(Circle(position, rvo_configuration.radius), mc.circle,
			v_nominal, mc.velocity, &vo_1, &vo_2);
		constraints.push_back(vo_1);
	}

	// scale and shift the constraints
	Vec2 shift = -1.f*v_nominal;
	float scaling = 0.5f/(shift.norm() + rvo_configuration.v_max + 0.01f);
	for (auto& c : constraints)
	{
		c.shift(shift);
		c.rescale(scaling);
	}

	// solve the normalized problem
	Vec2 v_corrected;
	try
	{
		Vec2 scaled_shifted_solution = Geometry2D::DistanceMinimizer::IncrementalDistanceMinimization(constraints);
		v_corrected = scaled_shifted_solution/scaling - shift;
	}
	catch (Geometry2D::DistanceMinimizer::InfeasibilityException e)
	{
		v_corrected = Vec2(0.f, 0.f);
		std::cout << "Infeasible constraints" << std::endl;
	}

	// make a step with the found velocity
	position = position + dt*v_corrected;
	last_step_velocity = v_corrected;
}
#include "orca_style.hpp"
#include "distance_minimizer.hpp"

#include <iostream>

using Geometry2D::Vec2;
using Geometry2D::HalfPlane2;
using AdditionalPrimitives2D::Circle;
using Geometry2D::DistanceMinimizer;

//const float OrcaStyle::tau = 2.f;
const float OrcaStyle::delta = 0.05f;
const float OrcaStyle::v_limit = 2.f;

Vec2 OrcaStyle::avoid(const RDS::Simulator::Robot& robot,
	const std::vector<RDS::Simulator::Obstacle>& obstacles, int i, float time, float tau)
{
	std::vector<HalfPlane2> constraints;
	
	float rxx = std::cos(robot.orientation);
	float rxy = std::sin(robot.orientation);
	float ryx = -rxy;
	float ryy = rxx;
	for (auto& c : robot.shape)
	{
		Vec2 center_global = Vec2(rxx*c.center.x + ryx*c.center.y,
					rxy*c.center.x + ryy*c.center.y) + robot.position;
		constraints.push_back(createConstraint(center_global - obstacles[i].position,
			c.radius + obstacles[i].radius, tau));
	}
	for (int j = 0; j < obstacles.size(); j++)
	{
		if (i == j)
			continue;
		constraints.push_back(createConstraint(obstacles[j].position - obstacles[i].position,
			obstacles[j].radius + obstacles[i].radius, tau));
	}

	constraints.push_back(HalfPlane2(Vec2(1.f, 0.f), v_limit));
	constraints.push_back(HalfPlane2(Vec2(-1.f, 0.f), v_limit));
	constraints.push_back(HalfPlane2(Vec2(0.f, 1.f), v_limit));
	constraints.push_back(HalfPlane2(Vec2(0.f, -1.f), v_limit));

	Vec2 shift = -1.f*obstacles[i].motion_law(time, obstacles[i].position);
	if (obstacles[i].use_constant_motion_law)
		shift = -1.f*obstacles[i].constant_motion_law;
	if (shift.x > v_limit)
		shift.x = v_limit;
	else if (shift.x < -v_limit)
		shift.x = -v_limit;
	if (shift.y > v_limit)
		shift.y = v_limit;
	else if (shift.y < -v_limit)
		shift.y = -v_limit;

	float scaling = 1/(2.f*v_limit);

	for (auto& con : constraints)
		con.shift(shift).rescale(scaling);

	try
	{
		Vec2 solution = DistanceMinimizer::IncrementalDistanceMinimization(constraints);
		//std::cout << solution.x << ", " << solution.y << std::endl;
		return solution/scaling - shift;
	}
	catch (DistanceMinimizer::InfeasibilityException e)
	{
		//std::cout << "Infeasible" << std::endl;
		return Vec2(0.f, 0.f);
	}
}

HalfPlane2 OrcaStyle::createConstraint(const Vec2& relative_position,
	float radius_sum, float tau)
{
	if (relative_position.norm() - radius_sum - delta > 0.f)
		return HalfPlane2(relative_position, (relative_position.norm() - radius_sum - delta)/2.f/tau);
	else
	{
		//std::cout << "Keeping zero-velocity constraint-free." << std::endl;
		return HalfPlane2(relative_position, 0.f);
	}
}
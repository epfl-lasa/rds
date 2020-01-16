#ifndef ORCA_STYLE_HPP
#define ORCA_STYLE_HPP

#include "geometry.hpp"
#include "simulator.hpp"

struct OrcaStyle
{
	static Geometry2D::Vec2 avoid(const RDS::Simulator::Robot& robot,
		const std::vector<RDS::Simulator::Obstacle>& obstacles, int i, float time, float tau,
		const std::vector<RDS::Simulator::Obstacle>& static_obstacles);

	static const float delta, v_limit;

private:
	static Geometry2D::HalfPlane2 createConstraint(const Geometry2D::Vec2& relative_position,
		float radius_sum, float tau);
};

#endif
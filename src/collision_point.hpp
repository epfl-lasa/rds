#ifndef COLLISION_POINT_HPP
#define COLLISION_POINT_HPP

#include "geometry.hpp"

namespace RDS
{
	struct CollisionPoint
	{
		Geometry2D::Vec2 p; // point on the robot
		Geometry2D::Vec2 p_to_q; // vector connecting p to its closest point q on a given obstacle
		Geometry2D::Vec2 v_q;	// q's velocity

		CollisionPoint(const Geometry2D::Vec2& p, const Geometry2D::Vec2& p_to_q, const Geometry2D::Vec2& v_q)
			: p(p)
			, p_to_q(p_to_q)
			, v_q(v_q)
		{ }

		Geometry2D::HalfPlane2 createPointPVelocityConstraint(float tau = 2.f, float delta = 0.1f) const
		{
			return Geometry2D::HalfPlane2(p_to_q, 1.f).rescale(1.f/tau*(p_to_q.norm() - delta) + 
				v_q.dot(p_to_q.normalized()));
		}
	};
}

#endif
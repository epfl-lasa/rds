#ifndef COLLISION_POINT_HPP
#define COLLISION_POINT_HPP

#include "geometry.hpp"

#include <cmath>

namespace RDS
{
	struct CollisionPoint
	{
		Geometry2D::Vec2 p; // point on the robot
		Geometry2D::Vec2 p_to_q; // vector connecting p to its closest point q on a given obstacle
		Geometry2D::Vec2 v_q;	// q's velocity

		CollisionPoint() { }

		CollisionPoint(const Geometry2D::Vec2& p, const Geometry2D::Vec2& p_to_q, const Geometry2D::Vec2& v_q)
			: p(p)
			, p_to_q(p_to_q)
			, v_q(v_q)
		{ }

		CollisionPoint(const AdditionalPrimitives2D::Circle& c_p, const AdditionalPrimitives2D::Circle& c_q, const Geometry2D::Vec2& c_q_v)
		{
			Geometry2D::Vec2 n_p_to_q((c_q.center - c_p.center).normalized());
			p = c_p.center + c_p.radius*n_p_to_q;
			float d = (c_q.center - c_p.center).norm() - c_p.radius - c_q.radius;
			//p_to_q = (d > 0.f) ? n_p_to_q*d : throw CollisionException();
			p_to_q = n_p_to_q*d;
			v_q = c_q_v;
		}

		CollisionPoint(const AdditionalPrimitives2D::Circle& c_p, const Geometry2D::Vec2& p_q, const Geometry2D::Vec2& c_q_v)
		{
			Geometry2D::Vec2 n_p_to_q((p_q - c_p.center).normalized());
			p = c_p.center + c_p.radius*n_p_to_q;
			float d = (p_q - c_p.center).norm() - c_p.radius;
			//p_to_q = (d > 0.f) ? n_p_to_q*d : throw CollisionException();
			p_to_q = n_p_to_q*d;
			v_q = c_q_v;
		}

		Geometry2D::HalfPlane2 createPointPVelocityConstraint(float tau = 2.f, float delta = 0.1f,
			bool unilateral_velocity_shift = false, bool velocity_obstacle_style = false) const
		{
			if (!velocity_obstacle_style)
			{
				//return Geometry2D::HalfPlane2(p_to_q, 1.f).rescale(1.f/tau*(p_to_q.norm() - delta) + 
				//	v_q.dot(p_to_q.normalized()));
				if (unilateral_velocity_shift && (v_q.dot(p_to_q.normalized()) < 0.f))
				{
					if (p_to_q.norm() > delta)
					{
						return Geometry2D::HalfPlane2(p_to_q, 1.f).rescale(1.f/tau*(std::sqrt(p_to_q.norm() - delta)));
					}
					
					return Geometry2D::HalfPlane2(p_to_q, 1.f).rescale(0.f);
				}
				else
				{
					if (p_to_q.norm() > delta)
					{
						return Geometry2D::HalfPlane2(p_to_q, 1.f).rescale(1.f/tau*(std::sqrt(p_to_q.norm() - delta)) + 
										v_q.dot(p_to_q.normalized()));
					}
					
					return Geometry2D::HalfPlane2(p_to_q, 1.f).rescale(v_q.dot(p_to_q.normalized()));
				}
			}
			else
			{
				// vo style
				// ...
			}
		}

		class CollisionException { };
	};
}

#endif
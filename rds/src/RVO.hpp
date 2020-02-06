#ifndef RVO_HPP
#define RVO_HPP

#include "geometry.hpp"

struct RVO
{
	RVO(float tau, float delta) : tau(tau), delta(delta) { }

	void computeCoordinativeVelocityObstacles(const AdditionalPrimitives2D::Circle& o_1,
		const AdditionalPrimitives2D::Circle& o_2,
		const Geometry2D::Vec2& vpref_1, const Geometry2D::Vec2& vpref_2,
		Geometry2D::HalfPlane2* vo_1, Geometry2D::HalfPlane2* vo_2);

	float getTau() const
	{
		return tau;
	}

	float getDelta() const
	{
		return delta;
	}

	Geometry2D::HalfPlane2 m_convex_rvo;
private:
	void computeConvexRVO(const Geometry2D::Vec2& relative_position,
		const Geometry2D::Vec2& relative_velocity_pref, float radius_sum,
		Geometry2D::HalfPlane2* crvo);

	float tau, delta;
};

#endif
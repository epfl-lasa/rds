#ifndef CAPSULE_HPP
#define CAPSULE_HPP

#include "geometry.hpp"

namespace Geometry2D
{
	struct Capsule
	{
		Capsule();
		Capsule(float radius, const Vec2& center_a, const Vec2& center_b);

		void closestMidLineSegmentPoint(const Vec2& pt_query, Vec2* pt_segment) const;

		const float& radius() const { return m_radius; }

		const Vec2& center_a() const { return m_center_a; }

		const Vec2& center_b() const { return m_center_b; }

		const HalfPlane2& bound_a() const { return m_bound_a; }

		const HalfPlane2& bound_b() const { return m_bound_b; }

	private:
		float m_radius;
		Vec2 m_center_a, m_center_b;
		HalfPlane2 m_bound_a, m_bound_b;
	};
}

#endif
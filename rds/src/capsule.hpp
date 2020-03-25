#ifndef CAPSULE_HPP
#define CAPSULE_HPP

#include "geometry.hpp"
#include <vector>

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

	struct BoundingCircles
	{
		BoundingCircles(unsigned int n_splits_mid_segment) : m_circles(n_splits_mid_segment + 3) { }

		void fit(const Capsule& capsule, float colliding_objects_radius);

		const std::vector<AdditionalPrimitives2D::Circle>& circles() const { return m_circles; }
	private:
		std::vector<AdditionalPrimitives2D::Circle> m_circles;

	};
}

#endif
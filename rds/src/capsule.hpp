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

		float radius;
		Vec2 center_a, center_b;
		HalfPlane2 bound_a, bound_b;
	};
}

#endif
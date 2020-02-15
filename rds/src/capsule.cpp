#include "capsule.hpp"

namespace Geometry2D
{
	Capsule::Capsule()
	: radius(0.5f)
	, center_a(0.f, -0.5f)
	, center_b(0.f, 0.5f)
	, bound_a(center_a - center_b, center_a.dot((center_a - center_b).normalized()))
	, bound_b(center_b - center_a, center_b.dot((center_b - center_a).normalized()))
	{ }

	Capsule::Capsule(float radius, const Vec2& center_a, const Vec2& center_b)
	: radius(radius)
	, center_a(center_a)
	, center_b(center_b)
	, bound_a(center_a - center_b, center_a.dot((center_a - center_b).normalized()))
	, bound_b(center_b - center_a, center_b.dot((center_b - center_a).normalized()))
	{ }

	void Capsule::closestMidLineSegmentPoint(const Vec2& pt_query, Vec2* pt_segment) const
	{
		if (bound_a.signedDistance(pt_query) > 0.f)
			*pt_segment = center_a;
		else if (bound_b.signedDistance(pt_query) > 0.f)
			*pt_segment = center_b;
		else
			*pt_segment = center_a + bound_a.getNormal()*bound_a.signedDistance(pt_query);
	}
}
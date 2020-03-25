#include "capsule.hpp"

namespace Geometry2D
{
	Capsule::Capsule()
	: m_radius(0.5f)
	, m_center_a(0.f, -0.5f)
	, m_center_b(0.f, 0.5f)
	, m_bound_a(m_center_a - m_center_b, m_center_a.dot((m_center_a - m_center_b).normalized()))
	, m_bound_b(m_center_b - m_center_a, m_center_b.dot((m_center_b - m_center_a).normalized()))
	{ }

	Capsule::Capsule(float radius, const Vec2& center_a, const Vec2& center_b)
	: m_radius(radius)
	, m_center_a(center_a)
	, m_center_b(center_b)
	, m_bound_a(center_a - center_b, center_a.dot((center_a - center_b).normalized()))
	, m_bound_b(center_b - center_a, center_b.dot((center_b - center_a).normalized()))
	{ }

	void Capsule::closestMidLineSegmentPoint(const Vec2& pt_query, Vec2* pt_segment) const
	{
		if (m_bound_a.signedDistance(pt_query) > 0.f)
			*pt_segment = m_center_a;
		else if (m_bound_b.signedDistance(pt_query) > 0.f)
			*pt_segment = m_center_b;
		else
			*pt_segment = m_center_a + m_bound_a.getNormal()*m_bound_a.signedDistance(pt_query);
	}
}
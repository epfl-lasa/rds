#include "capsule.hpp"
#include <cmath>

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

	void BoundingCircles::fit(const Capsule& capsule, float colliding_objects_radius)
	{
		unsigned int n_splits_mid_segment = m_circles.size() - 3;
		if (n_splits_mid_segment < 0)
		{
			m_circles.resize(0);
			return;
		}
		m_circles.front() = AdditionalPrimitives2D::Circle(capsule.center_a(), capsule.radius());
		m_circles.back() = AdditionalPrimitives2D::Circle(capsule.center_b(), capsule.radius());
		float A = (capsule.center_a() - capsule.center_b()).norm()/(n_splits_mid_segment + 1)/2.f;
		float B = capsule.radius() + colliding_objects_radius;
		float C = std::sqrt(A*A + B*B);
		float radius = C - colliding_objects_radius;
		for (unsigned int i = 1; i < m_circles.size() - 1; i++)
		{
			float fraction_position_from_a_to_b = (i - 0.5f)*1.f/(n_splits_mid_segment + 1);
			Vec2 center((1.f - fraction_position_from_a_to_b)*capsule.center_a() + 
				fraction_position_from_a_to_b*capsule.center_b());
			m_circles[i] = AdditionalPrimitives2D::Circle(center, radius);
		}
	}
}
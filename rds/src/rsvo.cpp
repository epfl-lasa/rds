#include "rsvo.hpp"
#define _USE_MATH_DEFINES
#include <cmath>

namespace Geometry2D
{
/*static float cross(const Vec2& v1, const Vec2& v2)
{
	return v1.x*v2.y - v1.y*v2.x;
}*/

int RSVO::determineSweepingAngles(const Vec2& v_nominal_front_cap)
{
	float w_nominal = -v_nominal_front_cap.x/m_capsule.center_a().y;

	m_sweeping_angle_clockwise = m_tau*(w_nominal - m_w_band_width/2.f);
	m_sweeping_angle_counter_clockwise = m_tau*(w_nominal + m_w_band_width/2.f);
	if (w_nominal < 0.f && m_sweeping_angle_counter_clockwise < 0.f)
		m_sweeping_angle_counter_clockwise = 0.f;
	if (w_nominal > 0.f && m_sweeping_angle_clockwise > 0.f)
		m_sweeping_angle_clockwise = 0.f;

	float combined_radius = m_capsule.radius() + m_object.radius;
	Vec2 relative_position = m_object.center - m_capsule.center_a();
	float front_cap_object_distance = relative_position.norm() - combined_radius;
	float mid_segment_length = (m_capsule.center_a() - m_capsule.center_b()).norm();
	if (front_cap_object_distance > mid_segment_length)
		return 0;

	Vec2 p_tangent_clockwise, p_tangent_counter_clockwise;
	if (0 != computeCircleTangents(relative_position, combined_radius,
		&p_tangent_clockwise, &p_tangent_counter_clockwise))
		return 1;
	HalfPlane2 h_tangent_clockwise(Vec2(-p_tangent_clockwise.y, p_tangent_clockwise.x), 0.f);
	HalfPlane2 h_tangent_counter_clockwise(Vec2(p_tangent_counter_clockwise.y, -p_tangent_counter_clockwise.x), 0.f);
	HalfPlane2 h_orthogonal(-1.f*(p_tangent_clockwise + p_tangent_counter_clockwise), 0.f);
	Vec2 e_mid_segment_clockwise(std::cos(m_sweeping_angle_clockwise - M_PI/2.f),
		std::sin(m_sweeping_angle_clockwise - M_PI/2.f));
	Vec2 e_mid_segment_counter_clockwise(std::cos(m_sweeping_angle_counter_clockwise - M_PI/2.f),
		std::sin(m_sweeping_angle_counter_clockwise - M_PI/2.f));
	
	if (h_tangent_clockwise.signedDistance(e_mid_segment_counter_clockwise) > 0.f)
	{
		
	}


	//if ((h_tangent_clockwise.signedDistance(e_mid_segment_counter_clockwise) > 0.f ||
	//	h_tangent_counter_clockwise.signedDistance(e_mid_segment_clockwise) > 0.f)
	//	return 0;
}

int RSVO::computeCircleTangents(const Vec2& center, float radius,
	Vec2* p_tangent_clockwise, Vec2* p_tangent_counter_clockwise)
{
	float v_c = center.norm() - radius;
	float v_r = radius;
	float r_thales_circle = 0.5f*(v_c + v_r);
	float tangent_point_1st_coord = r_thales_circle - v_r*v_r/2.f/r_thales_circle;
	float tangent_point_2nd_coord_squared = r_thales_circle*r_thales_circle - 
		tangent_point_1st_coord*tangent_point_1st_coord;
	if (tangent_point_2nd_coord_squared <= 0.f)
		return -1;

	Vec2 e_basis_1(center.normalized());
	Vec2 e_basis_2(-e_basis_1.y, e_basis_1.x);
	Vec2 c_thales_circle(center.normalized()*r_thales_circle);
	*p_tangent_counter_clockwise = Vec2(c_thales_circle + e_basis_1*tangent_point_1st_coord +
		e_basis_2*std::sqrt(tangent_point_2nd_coord_squared));
	*p_tangent_clockwise = Vec2(c_thales_circle + e_basis_1*tangent_point_1st_coord -
		e_basis_2*std::sqrt(tangent_point_2nd_coord_squared));
	return 0;
}

}
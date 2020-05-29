#include "RVO.hpp"

#define _USE_MATH_DEFINES
#include <cmath>

using Geometry2D::Vec2;
using Geometry2D::HalfPlane2;
using AdditionalPrimitives2D::Circle;

enum struct Boundary {plus, minus, arc};

void RVO::computeCoordinativeVelocityObstacles(const Circle& o_1, const Circle& o_2,
	const Vec2& vpref_1, const Vec2& vpref_2, HalfPlane2* vo_1, HalfPlane2* vo_2)
{
	Vec2 relative_position(o_2.center - o_1.center);
	Vec2 relative_velocity_pref(vpref_1 - vpref_2);
	float radius_sum = o_1.radius + o_2.radius + delta;
	HalfPlane2 convex_rvo;
	computeConvexRVO(relative_position, relative_velocity_pref, radius_sum, &convex_rvo);

	Vec2 shift_2(0.5*convex_rvo.signedDistance(relative_velocity_pref)*convex_rvo.getNormal());
	Vec2 shift_1(-1.f*shift_2);

	//*vo_1 = HalfPlane2(convex_rvo).shift(shift_1 + vpref_2);
	//*vo_2 = HalfPlane2(-1.f*convex_rvo.getNormal(), convex_rvo.getOffset()).shift(shift_2 + vpref_1);

	*vo_1 = HalfPlane2(convex_rvo.getNormal(), 0.f);
	vo_1->shift(vpref_1 + (-1.f)*vo_1->getNormal()*0.5f*convex_rvo.signedDistance(relative_velocity_pref));

	*vo_2 = HalfPlane2((-1.f)*convex_rvo.getNormal(), 0.f);
	vo_2->shift(vpref_2 + (-1.f)*vo_2->getNormal()*0.5f*convex_rvo.signedDistance(relative_velocity_pref));

	if (vo_1->getOffset() < 0.f)
	{
		Vec2 shift_to_free_origin(-1.f*vo_1->getNormal()*vo_1->getOffset());
		vo_1->shift(shift_to_free_origin);
		vo_2->shift(shift_to_free_origin);//*1.f/(1.f + 0.5f*convex_rvo.getOffset()));
		//if (vo_2->getOffset() < 0.f)
		//	vo_2->shift(-1.f*vo_2->getNormal()*vo_2->getOffset());
	}
	else if (vo_2->getOffset() < 0.f)
	{
		Vec2 shift_to_free_origin(-1.f*vo_2->getNormal()*vo_2->getOffset());
		vo_2->shift(shift_to_free_origin);
		vo_1->shift(shift_to_free_origin);//*1.f/(1.f + 0.5f*convex_rvo.getOffset()));
		//if (vo_1->getOffset() < 0.f)
		//	vo_1->shift(-1.f*vo_1->getNormal()*vo_1->getOffset());
	}
	m_convex_rvo = convex_rvo;
}

void RVO::computeConvexRVO(const Vec2& relative_position, const Vec2& relative_velocity_pref,
	float radius_sum, HalfPlane2* crvo, bool orca_style_convex)
{
	float centers_distance = relative_position.norm();
	float v_c = (centers_distance - radius_sum)/tau;

	if (v_c <= 0.f)
	{
		*crvo = HalfPlane2(relative_position, 0.f);
		return;
	}

	float v_r = radius_sum/tau;

	// compute the tightest pair of halfplanes whose intersection contains the RVO-cone
	float r_thales_circle = 0.5f*(v_c + v_r);
	float tangent_point_1st_coord = r_thales_circle - v_r*v_r/2.f/r_thales_circle;
	float tangent_point_2nd_coord_squared = r_thales_circle*r_thales_circle - 
		tangent_point_1st_coord*tangent_point_1st_coord;
	if (tangent_point_2nd_coord_squared <= 0.f)
	{
		*crvo = HalfPlane2(relative_position, 0.f);
		return;
	}

	Vec2 e_basis_1(relative_position.normalized());
	Vec2 e_basis_2(-e_basis_1.y, e_basis_1.x);
	Vec2 c_thales_circle(relative_position.normalized()*r_thales_circle);
	Vec2 p_tangent_plus(c_thales_circle + e_basis_1*tangent_point_1st_coord +
		e_basis_2*std::sqrt(tangent_point_2nd_coord_squared));
	Vec2 p_tangent_minus(c_thales_circle + e_basis_1*tangent_point_1st_coord -
		e_basis_2*std::sqrt(tangent_point_2nd_coord_squared));

	Vec2 c_cone_cap(relative_position.normalized()*(v_c + v_r));
	HalfPlane2 h_minus(p_tangent_minus - c_cone_cap, 0.f);
	HalfPlane2 h_plus(p_tangent_plus - c_cone_cap, 0.f);

	// check if v_pref is outside RVO
	HalfPlane2 h_minus_shifted(h_minus);
	h_minus_shifted.shift(-1.f*h_minus.getNormal()*v_r);
	HalfPlane2 h_plus_shifted(h_plus);
	h_plus_shifted.shift(-1.f*h_plus.getNormal()*v_r);
	Boundary candidates_boundary;
	Vec2 candidate;
	if (h_minus_shifted.signedDistance(relative_velocity_pref) <= 0.f)
		candidate = relative_velocity_pref;
	else
	{
		candidate = relative_velocity_pref - h_minus_shifted.getNormal()*
			h_minus_shifted.signedDistance(relative_velocity_pref);
		candidates_boundary = Boundary::minus;
	}
	if (h_plus_shifted.signedDistance(candidate) > 0.f)
	{
		candidate = relative_velocity_pref - h_plus_shifted.getNormal()*
			h_plus_shifted.signedDistance(relative_velocity_pref);
		candidates_boundary = Boundary::plus;
		if (h_minus_shifted.signedDistance(candidate) > 0.f)
		{
			candidate = c_cone_cap;
			candidates_boundary = Boundary::arc;
		}
	}

	if ((candidate - relative_velocity_pref).norm() >= v_r)
	{
		if (candidates_boundary == Boundary::minus)
		{
			*crvo = h_minus.flip();
			return;
		}
		if (candidates_boundary == Boundary::plus)
		{
			*crvo = h_plus.flip();
			return;
		}
		if (candidates_boundary == Boundary::arc)
		{
			*crvo = HalfPlane2(c_cone_cap - relative_velocity_pref, 0.f);
			crvo->shift(crvo->getNormal()*crvo->signedDistance(c_cone_cap -
				v_r*crvo->getNormal()));
			return;
		}
	}

	// v_pref is inside the RVO
	// -> intersect the ray through the origin and v_pref with the cap
	if (!orca_style_convex)
	{
		float v_tol = 1e-10f;
		Vec2 tangent_point(0.f, 0.f);
		if (v_tol < relative_velocity_pref.norm())
		{
			float discriminant = 4.f*(relative_velocity_pref.dot(c_cone_cap)*relative_velocity_pref.dot(c_cone_cap) -
				relative_velocity_pref.dot(relative_velocity_pref)*(c_cone_cap.dot(c_cone_cap) - v_r*v_r));
			if (discriminant > 0.f)
			{
				float scaling = (2.f*relative_velocity_pref.dot(c_cone_cap) - std::sqrt(discriminant))/
					2.f/relative_velocity_pref.dot(relative_velocity_pref);
				tangent_point = relative_velocity_pref*scaling;
			}
			else
			{
				float scaling = relative_velocity_pref.dot(c_cone_cap)/
					relative_velocity_pref.dot(relative_velocity_pref);
				tangent_point = relative_velocity_pref*scaling;
			}
		}
		// compute the tangent halfplane
		*crvo = HalfPlane2(c_cone_cap - tangent_point, 0.f);
		crvo->shift(crvo->getNormal()*crvo->signedDistance(tangent_point));
		return;
	}
	else
	{
		HalfPlane2 h_cut_cap(Vec2(0.f, 0.f), p_tangent_plus, p_tangent_minus);
		Vec2 tangent_point_cap = c_cone_cap + v_r*(relative_velocity_pref - c_cone_cap).normalized();
		if (0.f > h_cut_cap.signedDistance(tangent_point_cap))
		{
			*crvo = HalfPlane2(c_cone_cap - tangent_point_cap, 0.f);
			crvo->shift(crvo->getNormal()*crvo->signedDistance(tangent_point_cap));
		}
		else if (h_plus.signedDistance(relative_velocity_pref) > h_minus.signedDistance(relative_velocity_pref))
			*crvo = h_plus;
		else
			*crvo = h_minus;
	}
}
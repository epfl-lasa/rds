#include "cvo.hpp"
#define _USE_MATH_DEFINES
#include <cmath>

namespace Geometry2D
{
CVO::CVO(const Vec2& own_position, const Vec2& obj_position,
	const Vec2& v_opt, const Vec2& v_obj,
	float radius_sum, float tau)
{
	Vec2 relative_position = obj_position - own_position;
	float centers_distance = relative_position.norm();
	float v_c = (centers_distance - radius_sum)/tau;

	if (v_c <= 0.f)
	{
		conservative_constraint = HalfPlane2(relative_position, 0.f);
		return;
	}

	v_r = radius_sum/tau;
	c_cone_cap = 1.f/tau*relative_position;

	Vec2 p_tangent_minus, p_tangent_plus;
	if (computeCircleTangents(c_cone_cap, v_r, &p_tangent_minus, &p_tangent_plus) != 0)
	{
		conservative_constraint = HalfPlane2(relative_position, 0.f);
		return;
	}

	h_minus = HalfPlane2(p_tangent_minus - c_cone_cap, 0.f);
	h_plus = HalfPlane2(p_tangent_plus - c_cone_cap, 0.f);
	h_minus_shifted = HalfPlane2(h_minus);
	h_minus_shifted.shift(-1.f*h_minus.getNormal()*v_r);
	h_plus_shifted = HalfPlane2(h_plus);
	h_plus_shifted.shift(-1.f*h_plus.getNormal()*v_r);

	Vec2 minus_v_obj_projection;
	bool minus_v_obj_is_inside;
	Boundary minus_v_obj_proj_boundary;
	projectOnRelativeVOBoundary(-1.f*v_obj, &minus_v_obj_projection, &minus_v_obj_is_inside, &minus_v_obj_proj_boundary);

	if (minus_v_obj_is_inside)
	{
		if (minus_v_obj_proj_boundary == Boundary::arc)
			conservative_constraint = HalfPlane2(c_cone_cap - minus_v_obj_projection, 0.f);
		else if (minus_v_obj_proj_boundary == Boundary::plus)
			conservative_constraint = HalfPlane2(-1.f*h_plus.getNormal(), 0.f);
		else
			conservative_constraint = HalfPlane2(-1.f*h_minus.getNormal(), 0.f);
		return;
	}

	Vec2 v_rel_opt = v_opt - v_obj;
	Vec2 v_rel_opt_projection;
	Boundary v_rel_opt_proj_boundary;
	projectOnRelativeVOBoundary(v_rel_opt, &v_rel_opt_projection, 0, &v_rel_opt_proj_boundary);

	if (v_rel_opt_proj_boundary == Boundary::arc)
	{
		conservative_constraint = HalfPlane2(c_cone_cap - v_rel_opt_projection,
			v_rel_opt_projection.dot((c_cone_cap - v_rel_opt_projection).normalized()));
	}
	else if (v_rel_opt_proj_boundary == Boundary::plus)
		conservative_constraint = HalfPlane2(-1.f*h_plus.getNormal(), v_rel_opt_projection.dot(-1.f*h_plus.getNormal()));
	else
		conservative_constraint = HalfPlane2(-1.f*h_minus.getNormal(), v_rel_opt_projection.dot(-1.f*h_minus.getNormal()));

	conservative_constraint.shift(v_obj);

	if (conservative_constraint.getOffset() > 0.f)
		return;

	Vec2 absolute_VO_center = c_cone_cap + v_obj;
	Vec2 p_tangent_clockwise, p_tangent_counter_clockwise;
	if (computeCircleTangents(absolute_VO_center, v_r, &p_tangent_clockwise, &p_tangent_counter_clockwise) != 0)
	{
		if (minus_v_obj_proj_boundary == Boundary::arc)
			conservative_constraint = HalfPlane2(c_cone_cap - minus_v_obj_projection, 0.f);
		else if (minus_v_obj_proj_boundary == Boundary::plus)
			conservative_constraint = HalfPlane2(-1.f*h_plus.getNormal(), 0.f);
		else
			conservative_constraint = HalfPlane2(-1.f*h_minus.getNormal(), 0.f);
		return;
	}

	float cross_product = relative_position.x*v_obj.y - relative_position.y*v_obj.x;
	if (cross_product > 0.f)
		conservative_constraint = HalfPlane2(absolute_VO_center, p_tangent_counter_clockwise, Vec2(0.f, 0.f));
	else
		conservative_constraint = HalfPlane2(absolute_VO_center, p_tangent_clockwise, Vec2(0.f, 0.f));
}

int CVO::computeCircleTangents(const Vec2& center, float radius, Vec2* p_tangent_clockwise, Vec2* p_tangent_counter_clockwise)
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

void CVO::projectOnRelativeVOBoundary(const Vec2& relative_velocity, Vec2* projection, bool* is_inside_VO, Boundary* boundary)
{
	Boundary candidates_boundary = Boundary::any;
	Vec2 candidate = relative_velocity;
	if (h_minus_shifted.signedDistance(candidate) > 0.f)
	{
		candidate = relative_velocity - h_minus_shifted.getNormal()*
			h_minus_shifted.signedDistance(relative_velocity);
		candidates_boundary =  Boundary::minus;
	}
	if (h_plus_shifted.signedDistance(candidate) > 0.f)
	{
		candidate = relative_velocity - h_plus_shifted.getNormal()*
			h_plus_shifted.signedDistance(relative_velocity);
		candidates_boundary =  Boundary::plus;
		if (h_minus_shifted.signedDistance(candidate) > 0.f)
		{
			candidate = c_cone_cap;
			candidates_boundary =  Boundary::arc;
		}
	}

	if ((candidate - relative_velocity).norm() >= v_r)
	{
		if (is_inside_VO != 0)
			*is_inside_VO = false;
		*projection = candidate + v_r*(relative_velocity - candidate).normalized();
		*boundary = candidates_boundary;
		return;
	}
	else
	{
		if (is_inside_VO != 0)
			*is_inside_VO = true;
		if ((candidate - relative_velocity).norm() >= 0.000001f)
		{
			*projection = candidate + v_r*(relative_velocity - candidate).normalized();
			*boundary = candidates_boundary;
		}
		else if (h_minus.signedDistance(relative_velocity) > h_plus.signedDistance(relative_velocity))
		{
			*projection = relative_velocity - h_minus.signedDistance(relative_velocity)*h_minus.getNormal();
			*boundary = Boundary::minus;
		}
		else
		{
			*projection = relative_velocity - h_plus.signedDistance(relative_velocity)*h_plus.getNormal();
			*boundary = Boundary::plus;
		}
		return;
	}
}

}
#ifndef CVO_HPP
#define CVO_HPP

#include "geometry.hpp"

namespace Geometry2D
{
struct CVO
{
	CVO(const Vec2& own_position, const Vec2& obj_position,
		const Vec2& v_opt, const Vec2& v_obj,
		float radius_sum, float tau);

	const HalfPlane2& getConservativeConstraint() const { return conservative_constraint; }
	const Vec2& getRelativeVOCenter() const { return c_cone_cap; }
	float getVORadius() const { return v_r; }
	const HalfPlane2& getRelativeVOHPlus() const { return h_plus; }
	const HalfPlane2& getRelativeVOHMinus() const { return h_minus; }

private:
	enum struct Boundary {plus, minus, arc, any};

	static int computeCircleTangents(const Vec2& center, float radius,
		Vec2* p_tangent_clockwise, Vec2* p_tangent_counter_clockwise);

	void projectOnRelativeVOBoundary(const Vec2& relative_velocity,
		Vec2* projection, bool* is_inside_VO, Boundary* boundary);


	HalfPlane2 conservative_constraint;
	float v_r;
	Vec2 c_cone_cap;
	HalfPlane2 h_plus, h_minus, h_plus_shifted, h_minus_shifted;
};
}

#endif
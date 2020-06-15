#ifndef CVO_HPP
#define CVO_HPP

#include "geometry.hpp"

namespace Geometry2D
{
class CVO
{
	CVO(const Vec2& own_position, const Vec2& obj_position,
		const Vec2& v_opt, const Vec2& v_obj,
		float radius_sum, float tau);

	HalfPlane2 conservative_constraint;
};
}

#endif
#include "geometry.hpp"

#include <cmath>

namespace Geometry2D
{
	bool divisionIsOutsideRange(float numerator, float denominator)
	{
		if (std::abs(range*denominator) < std::abs(numerator))
			return true;
		else
			return false;
	}

	bool divisionIsInRange(float numerator, float denominator, float range_a, float range_b)
	{
		if (denominator > 0.0)
		{
			if (range_a > range_b)
			{
				if (range_a*denominator >= numerator &&
					range_b*denominator <= numerator)
					return true;
				else
					return false;
			}
			else
			{
				if (range_b*denominator >= numerator &&
					range_a*denominator <= numerator)
					return true;
				else
					return false;
			}
		}
		else if (denominator < 0.0)
		{
			if (range_a > range_b)
			{
				if (range_a*denominator <= numerator &&
					range_b*denominator >= numerator)
					return true;
				else
					return false;
			}
			else
			{
				if (range_b*denominator <= numerator &&
					range_a*denominator >= numerator)
					return true;
				else
					return false;
			}
		}
		else
			return false;
	}

/*	bool divisionIsOutsideRange(float numerator, float denominator)
	{
		if (((numerator > 0) && (denominator > 0)) ||
			((numerator < 0) && (denominator < 0)))
		{
			if (range*denominator < numerator)
				return true;
			else
				return false;
		}
		else
		{
			if (-range*denominator > numerator)
				return true;
			else
				return false;
		}
	}*/

	Vec2::Vec2() : x(0.0), y(0.0) {}

	Vec2::Vec2(float x, float y) : x(x), y(y) {}

	float Vec2::dot(const Vec2& v) const
	{
		return x*v.x + y*v.y;
	}

	float Vec2::norm() const
	{
		return std::sqrt(x*x + y*y);
	}

	Vec2 operator* (float s, const Vec2& v)
	{
		return Vec2(v.x*s, v.y*s);
	}

	Vec2 operator* (const Vec2& v, float s)
	{
		return Vec2(v.x*s, v.y*s);
	}

	Vec2 operator/ (const Vec2& v, float s)
	{
		return Vec2(v.x/s, v.y/s);
	}

	Vec2 operator+ (const Vec2& v1, const Vec2& v2)
	{
		return Vec2(v1.x + v2.x, v1.y + v2.y);
	}

	Vec2 operator- (const Vec2& v1, const Vec2& v2)
	{
		return Vec2(v1.x - v2.x, v1.y - v2.y);
	}

	Vec2 operator- (const Vec2& v)
	{
		return Vec2(-v.x,-v.y);
	}

	float incircleRadius(const Vec2& corner_a, const Vec2& corner_b, const Vec2& corner_c,
		Vec2* incircle_center)
	{
		float a = (corner_b - corner_c).norm();
		float b = (corner_c - corner_a).norm();
		float c = (corner_a - corner_b).norm();

		if (a < resolution)
		{
			if (incircle_center)
				*incircle_center = corner_a;
			return 0.0;
		}
		else if (b < resolution)
		{
			if (incircle_center)
				*incircle_center = corner_b;
			return 0.0;
		}
		else if (c < resolution)
		{
			if (incircle_center)
				*incircle_center = corner_c;
			return 0.0;
		}
		else
		{
			if (incircle_center)
				*incircle_center = (a*corner_a + b*corner_b + c*corner_c)/(a + b + c);
			float s = (a + b + c)/2.0;
			return std::sqrt((s - a)*(s - b)*(s - c)/s);
		}
	}

	Circle2 computeIncircle(const Vec2& corner_a, const Vec2& corner_b, const Vec2& corner_c)
	{
		float a = (corner_b - corner_c).norm();
		float b = (corner_c - corner_a).norm();
		float c = (corner_a - corner_b).norm();

		if (a < resolution)
			return Circle2(corner_a, 0.0);
		else if (b < resolution)
			return Circle2(corner_b, 0.0);
		else if (c < resolution)
			return Circle2(corner_c, 0.0);
		else
		{
			
			Vec2 center((a*corner_a + b*corner_b + c*corner_c)/(a + b + c));
			float s = (a + b + c)/2.0;
			float radius = ((s - a)*(s - b)*(s - c)/s);
			return Circle2(center, radius);
		}
	}

	Circle2::Circle2() : center(Vec2()), radius(0.0) {}

	Circle2::Circle2(Vec2 c, float r) : center(c), radius(r) {}

	HalfPlane2::HalfPlane2()
	{
		normal.x = 0.0;
		normal.y = 0.0;
		offset = 0.0;
	}

	HalfPlane2::HalfPlane2(const Vec2& normal, float offset) :
		normal(normal),
		offset(offset) {}

	const HalfPlane2& HalfPlane2::flip()
	{
		normal = -normal;
		offset = -offset;
		return *this;
	}

	bool HalfPlane2::contains(const Vec2& point_position_vector) const
	{
		if (point_position_vector.dot(normal) <= offset)
			return true;
		else
			return false;
	}

	Vec2 HalfPlane2::boundaryProjection(const Vec2& point_position_vector) const
	{
		return point_position_vector + (offset -
			point_position_vector.dot(normal))*normal;
	}

	Vec2 HalfPlane2::boundaryIntersection(const HalfPlane2& h, bool* outside_range) const
	{
		Vec2 anchor = offset*normal; // point on this half-plane's boundary
		Vec2 e_line(-normal.y, normal.x); // unit vector along this boundary
		// (anchor + lambda*e_line).dot(h.normal) == h.offset
		// => lambda = (h.offset - anchor.dot(h.normal))/e_line.dot(h.normal)
		float numerator = h.offset - anchor.dot(h.normal);
		float denominator = e_line.dot(h.normal);
		if (divisionIsOutsideRange(numerator, denominator))
		{
			*outside_range = true;
			return Vec2();
		}
		else
		{
			*outside_range = false;
			return anchor + numerator/denominator*e_line;
		}
	}

	AxisAlignedBoundingBox2::AxisAlignedBoundingBox2(float x_lower, float x_upper,
		float y_lower, float y_upper)
	:	x_lower(x_lower), x_upper(x_upper), y_lower(y_lower), y_upper(y_upper),
		h_x_lower(Vec2(-1.0,0.0), -x_lower), h_x_upper(Vec2(1.0,0.0), x_upper),
		h_y_lower(Vec2(0.0,-1.0), -y_lower), h_y_upper(Vec2(0.0,1.0), y_upper),
		c_x_lower_y_lower(Vec2(x_lower, y_lower)),
		c_x_upper_y_lower(Vec2(x_upper, y_lower)),
		c_x_lower_y_upper(Vec2(x_lower, y_upper)),
		c_x_upper_y_upper(Vec2(x_upper, y_upper))
	{}

	//AxisAlignedBoundingBox2::AxisAlignedBoundingBox2(const std::vector<Vec2>& points)

	AABBLineIntersection AxisAlignedBoundingBox2::intersectWithHalfPlaneBoundary(
		const HalfPlane2& h) const
	{
		// the half-plane boundary intersects the aabb if the corners lie on different sides
		// and similar considerations are used for the other cases
		if (h.contains(c_x_lower_y_lower))
		{
			if (h.contains(c_x_lower_y_upper))
			{
				if (h.contains(c_x_upper_y_lower))
				{
					if (h.contains(c_x_upper_y_upper))
					{
						return AABBLineIntersection::no_intersection;
					}
					else
					{
						return AABBLineIntersection::x_upper_y_upper;
					}
				}
				else
				{
					if (h.contains(c_x_upper_y_upper))
					{
						return AABBLineIntersection::x_upper_y_lower;
					}
					else
					{
						return AABBLineIntersection::y_lower_y_upper;
					}
				}
			}
			else
			{
				if (h.contains(c_x_upper_y_lower))
				{
					if (h.contains(c_x_upper_y_upper))
					{
						return AABBLineIntersection::x_lower_y_upper;
					}
					else
					{
						return AABBLineIntersection::x_lower_x_upper;
					}
				}
				else
				{
					return AABBLineIntersection::x_lower_y_lower;
				}
			}
		}
		else // here is the symmetry axis for the return statements
		{
			if (h.contains(c_x_lower_y_upper))
			{
				if (h.contains(c_x_upper_y_lower))
				{
					return AABBLineIntersection::x_lower_y_lower;
				}
				else
				{
					if (h.contains(c_x_upper_y_upper))
					{
						return AABBLineIntersection::x_lower_x_upper;
					}
					else
					{
						return AABBLineIntersection::x_lower_y_upper;
					}
				}
			}
			else
			{
				if (h.contains(c_x_upper_y_lower))
				{
					if (h.contains(c_x_upper_y_upper))
					{
						return AABBLineIntersection::y_lower_y_upper;
					}
					else
					{
						return AABBLineIntersection::x_upper_y_lower;
					}
				}
				else
				{
					if (h.contains(c_x_upper_y_upper))
					{
						return AABBLineIntersection::x_upper_y_upper;
					}
					else
					{
						return AABBLineIntersection::no_intersection;
					}
				}
			}
		}
	}

	// Checks if the intersection between this half-plane's boundary and the given other 
	// half-plane's boundary lies within the specified axis-aligned bounding box.
	bool HalfPlane2::checkIfBoundaryIntersectionIsInBoundingBox(const HalfPlane2& other_half_plane,
		const AxisAlignedBoundingBox2& aabb) const
	{
		AABBLineIntersection this_h_aabb = aabb.intersectWithHalfPlaneBoundary(*this);
		AABBLineIntersection other_h_aabb = aabb.intersectWithHalfPlaneBoundary(other_half_plane);
		if (this_h_aabb == AABBLineIntersection::no_intersection ||
			other_h_aabb == AABBLineIntersection::no_intersection)
		{
			return false;
		}
		else
		{
			Vec2 base_point;
			float lambda_end;
			if (this_h_aabb == AABBLineIntersection::x_lower_x_upper)
			{
				base_point = this->boundaryIntersection(aabb.h_x_lower);
				lambda_end = this->boundaryIntersection(aabb.h_x_upper, base_point);
			}
			else if (this_h_aabb == AABBLineIntersection::y_lower_y_upper)
			{
				base_point = this->boundaryIntersection(aabb.h_y_lower);
				lambda_end = this->boundaryIntersection(aabb.h_y_upper, base_point);
			}
			else if (this_h_aabb == AABBLineIntersection::x_lower_y_lower)
			{
				base_point = this->boundaryIntersection(aabb.h_x_lower);
				lambda_end = this->boundaryIntersection(aabb.h_y_lower, base_point);
			}
			else if (this_h_aabb == AABBLineIntersection::x_lower_y_upper)
			{
				base_point = this->boundaryIntersection(aabb.h_x_lower);
				lambda_end = this->boundaryIntersection(aabb.h_y_upper, base_point);
			}
			else if (this_h_aabb == AABBLineIntersection::x_upper_y_lower)
			{
				base_point = this->boundaryIntersection(aabb.h_x_upper);
				lambda_end = this->boundaryIntersection(aabb.h_y_lower, base_point);
			}
			else if (this_h_aabb == AABBLineIntersection::x_upper_y_upper)
			{
				base_point = this->boundaryIntersection(aabb.h_x_upper);
				lambda_end = this->boundaryIntersection(aabb.h_y_upper, base_point);
			}

			if (this->rangeCheckBoundaryIntersection(other_half_plane, base_point, 0.0, lambda_end))
				return true;
			else
				return false;
		}
	}
		/*	 ||
			(this_h_aabb == AABBLineIntersection::x_lower_y_lower && 
				other_h_aabb == AABBLineIntersection::x_upper_y_upper) || // this ignores line coincidence
			(this_h_aabb == AABBLineIntersection::x_upper_y_upper && 
				other_h_aabb == AABBLineIntersection::x_lower_y_lower) || // this ignores line coincidence
			(this_h_aabb == AABBLineIntersection::x_lower_y_upper && 
				other_h_aabb == AABBLineIntersection::x_upper_y_lower) || // this ignores line coincidence
			(this_h_aabb == AABBLineIntersection::x_upper_y_lower && 
				other_h_aabb == AABBLineIntersection::x_lower_y_upper)) // this ignores line coincidence
		{
			return false;
		}
		else if (this_h_aabb == AABBLineIntersection::x_lower_y_lower &&
			other_h_aabb == AABBLineIntersection::x_lower_y_lower)
		{

		}*/

	Vec2 HalfPlane2::boundaryIntersection(const HalfPlane2& h) const
	{
		Vec2 anchor = offset*normal; // point on this half-plane's boundary
		Vec2 e_line(-normal.y, normal.x); // unit vector along this boundary
		// (anchor + lambda*e_line).dot(h.normal) == h.offset
		// => lambda = (h.offset - anchor.dot(h.normal))/e_line.dot(h.normal)
		return anchor + (h.offset - anchor.dot(h.normal))/e_line.dot(h.normal)*e_line;
	}

	float HalfPlane2::boundaryIntersection(const HalfPlane2& h, const Vec2& base_point) const
	{
		Vec2 e_line(-normal.y, normal.x); // unit vector along this boundary
		// (anchor + lambda*e_line).dot(h.normal) == h.offset
		// => lambda = (h.offset - anchor.dot(h.normal))/e_line.dot(h.normal)
		return (h.offset - base_point.dot(h.normal))/e_line.dot(h.normal);
	}

	bool HalfPlane2::rangeCheckBoundaryIntersection(const HalfPlane2& h, const Vec2& base_point,
		float range_a, float range_b) const
	{
		Vec2 e_line(-normal.y, normal.x); // unit vector along this boundary
		// (anchor + lambda*e_line).dot(h.normal) == h.offset
		// => lambda = (h.offset - anchor.dot(h.normal))/e_line.dot(h.normal)
		float numerator = h.offset - base_point.dot(h.normal);
		float denominator = e_line.dot(h.normal);
		return divisionIsInRange(numerator, denominator, range_a, range_b);
	}

	// Shifts the point on this boundary to where both half-plane boundaries intersect.
	// Caution: it does not check if the problem is well-posed (so it might create
	// INF or NAN or crash for boundaries that are close to parallel). It returns the sign of
	// the shift, which is positive in the direction obtained by rotating this half-plane's normal 
	// by 90 degrees counter-clockwise.
	int HalfPlane2::shiftPointOnThisBoundaryToIntersection(const HalfPlane2& other_half_plane,
		Vec2* point_on_this_half_plane_boundary) const
	{
		Vec2 anchor = *point_on_this_half_plane_boundary; // point on this half-plane's boundary
		Vec2 e_line(-normal.y, normal.x); // unit vector along this boundary
		// (anchor + lambda*e_line).dot(h.normal) == h.offset
		// => lambda = (h.offset - anchor.dot(h.normal))/e_line.dot(h.normal)
		float lambda = (other_half_plane.offset - anchor.dot(other_half_plane.normal))/e_line.dot(
			other_half_plane.normal);
		*point_on_this_half_plane_boundary = anchor + lambda*e_line;
		if (lambda > 0.0)
			return 1;
		else if (lambda == 0.0)
			return 0;
		else
			return -1;
	}
}
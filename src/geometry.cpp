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

	HalfPlane2::HalfPlane2()
	{
		normal.x = 0.0;
		normal.y = 0.0;
		offset = 0.0;
	}

	HalfPlane2::HalfPlane2(const Vec2& normal, float offset) :
		normal(normal),
		offset(offset) {}

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
}
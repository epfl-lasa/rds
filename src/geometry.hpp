#ifndef  GEOMETRY_HPP
#define  GEOMETRY_HPP

#include <cmath>

namespace Geometry2D
{
	class Vec2 // cartesian vector in 2D
	{
	public:
		Vec2()
			: x(0.0)
			, y(0.0)
		{ }

		Vec2(float x, float y)
			: x(x)
			, y(y)
		{ }

		float norm() const
		{
			return std::sqrt(x*x + y*y);
		}

		float x, y;
	};

	inline Vec2 operator+ (const Vec2& v1, const Vec2& v2) // vector addition
	{
		return Vec2(v1.x + v2.x, v1.y + v2.y);
	}
	inline Vec2 operator- (const Vec2& v1, const Vec2& v2) // vector subtraction
	{
		return Vec2(v1.x - v2.x, v1.y - v2.y);
	}
	inline Vec2 operator* (float s, const Vec2& v) // multiply vector by scalar
	{
		return Vec2(v.x*s, v.y*s);
	}
	inline Vec2 operator* (const Vec2& v, float s) // multiply vector by scalar
	{
		return Vec2(v.x*s, v.y*s);
	}
	inline Vec2 operator/ (const Vec2& v, float s) // divide vector by scalar
	{
		return Vec2(v.x/s, v.y/s);
	}	
	inline float operator* (const Vec2& v1, const Vec2& v2) // inner product (dot product)
	{
		return v1.x*v2.x + v1.y*v2.y;
	}

	class HalfPlane2 // closed half-plane (the boundary line belongs to the half-plane)
	{
	public:
		HalfPlane2(const Vec2& normal_, float offset_)
			: normal(normal_)
			, offset(offset_)
			, parallel(-normal_.y, normal_.x)
		{
			if (normal_.norm() > 0.00000001f)
			{
				normal = normal/normal_.norm();
				parallel = parallel/normal_.norm();
			}
			else
				throw "Cannot normalize too small normal vector.";
		}

		const Vec2& getNormal() const
		{
			return normal;
		}

		const Vec2& getParallel() const
		{
			return parallel;
		}

		float getOffset() const
		{
			return offset;
		}

		// returns true if the point p belongs to the half-plane and false otherwise
		bool contains(const Vec2& p) const
		{
			return (p*normal <= offset);
		}

		// returns the point p's orthogonal projection on the half-plane's boundary
		Vec2 projectOnBoundary(const Vec2& p) const
		{
			return p + (offset - p*normal)*normal;
		}

		// returns the point obtained by shifting the given point p in the direction of
		// this half-plane's parallel vector by the given value shift
		Vec2 applyParallelShift(float shift, const Vec2& p) const
		{
			return p + shift*parallel;
		}

		// returns the (signed) shift in the direction of this half-plane's parallel vector
		// which the point p requires to arrive on the boundary of the other half-plane h
		// (not applicable when the half-planes' boundaries are parallel or close-to parallel)
		float getParallelShiftToAnotherBoundary(const Vec2& p, const HalfPlane2& h) const
		{
			return (h.offset - p*h.normal)/(this->parallel*h.normal);
		}

	private:
		Vec2 normal;
		float offset;
		Vec2 parallel;
	};
}

#endif

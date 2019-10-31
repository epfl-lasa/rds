#ifndef  GEOMETRY_HPP
#define  GEOMETRY_HPP

#include <cmath>

namespace Geometry2D
{
	class Vec2 // cartesian vector in 2D
	{
	public:
		Vec2()
			: x(0.f)
			, y(0.f)
		{ }

		Vec2(float x, float y)
			: x(x)
			, y(y)
		{ }

		float norm() const
		{
			return std::sqrt(x*x + y*y);
		}

		float dot(const Vec2& v) const // dot product (inner product)
		{
			return x*v.x + y*v.y;
		}

		Vec2 operator+ (const Vec2& v) const // vector addition
		{
			return Vec2(x + v.x, y + v.y);
		}
		
		Vec2 operator- (const Vec2& v) const // vector subtraction
		{
			return Vec2(x - v.x, y - v.y);
		}

		Vec2 operator* (float s) const // multiply vector by scalar
		{
			return Vec2(x*s, y*s);
		}

		Vec2 operator/(float s) const // divide vector by scalar
		{
			return Vec2(x/s, y/s);
		}

		Vec2 normalized() const // return normalized vector
		{
			return (norm() > 0.0000001f) ? *this/norm() : throw NormalizationException();
		}

		class NormalizationException { };

		float x, y;
	};

	inline Vec2 operator* (float s, const Vec2& v) // multiply vector by scalar
	{
		return Vec2(v.x*s, v.y*s);
	}

	class HalfPlane2 // closed half-plane (the boundary line belongs to the half-plane)
	{
	public:
		HalfPlane2(const Vec2& normal_unnormalized, float offset)
			: normal(normal_unnormalized.normalized())
			, offset(offset)
			, parallel(-normal.y, normal.x)
			, origo(offset*normal)
		{ }

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

		const Vec2& getOrigo() const
		{
			return origo;
		}
		// returns true if the point p belongs to the half-plane and false otherwise
		bool contains(const Vec2& p) const
		{
			return (p.dot(normal) <= offset);
		}
		// returns p's distance from the boundary (the sign is positive for points outside)
		float signedDistance(const Vec2& p) const
		{
			return p.dot(normal) - offset;
		}
		// returns the point p's orthogonal projection on the half-plane's boundary
		Vec2 projectOnBoundary(const Vec2& p) const
		{
			return p + (offset - p.dot(normal))*normal;
		}
		// returns the point obtained by shifting the given point p in the direction of
		// this half-plane's parallel vector by the given value shift
		Vec2 applyParallelShift(float shift, const Vec2& p) const
		{
			return p + shift*parallel;
		}
		// returns the (signed) shift in the direction of this half-plane's parallel vector
		// which the point p requires to arrive on the boundary of the other half-plane h
		// (caution when the half-planes' boundaries are parallel or close-to parallel)
		float getParallelShiftToAnotherBoundary(const Vec2& p, const HalfPlane2& h) const
		{
			return (h.offset - p.dot(h.normal))/(this->parallel.dot(h.normal));
		}

		Vec2 intersectBoundaries(const HalfPlane2& h) const
		{
			return origo + (h.offset - origo.dot(h.normal))/(parallel.dot(h.normal))*parallel;
		}
		//
		//bool checkBoundariesForParallelism()

	private:
		Vec2 normal;
		float offset;
		Vec2 parallel;
		Vec2 origo; // orthogonal projection of the point (0,0) on the halfplane's boundary
	};
/*
	class HVec3 // homogeneous coordinates for elements of the projective geometry in 2D
	{
	public:
		HVec3()
			: a(0.f)
			, b(0.f)
			, c(0.f)
		{ }
		
		HVec3(float a, b, c)
			: a(a)
			, b(b)
			, c(c)
		{ }

		HVec3(const Vec2& v) // initialize as the point denoted by v
			: a(v.x)
			, b(v.y)
			, c(1.f)
		{ }

		HVec3(const HalfPlane2& h) // initialize as the boundary line of h
			: a(h.normal.x)
			, b(h.normal.y)
			, c(-h.offset)
		{ }

		HVec3 cross(const HVec3& w) const
		{
			return HVec3(b*w.c - c*w.b, c*w.a - a*w.c, a*w.b - b*w.a);
		}

		float dot(const HVec3& w) const
		{
			return a*w.a + b*w.b + c*w.c;
		}

		float a, b, c;
	};
*/
}

#endif

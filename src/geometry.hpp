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
		HalfPlane2()
			: normal(Vec2(1.f, 0.f))
			, offset(0.f)
			, parallel(-normal.y, normal.x)
			, origo(offset*normal)
		{ }

		HalfPlane2(const Vec2& normal_unnormalized, float offset)
			: normal(normal_unnormalized.normalized())
			, offset(offset)
			, parallel(-normal.y, normal.x)
			, origo(offset*normal)
		{ }

		HalfPlane2(const Vec2& p_interior, const Vec2& p1_boundary, const Vec2& p2_boundary)
		{
			parallel = (p1_boundary - p2_boundary).normalized();
			normal = Vec2(parallel.y, -parallel.x);
			offset = normal.dot(p1_boundary);
			origo = offset*normal;
			if (signedDistance(p_interior) > 0.f)
				this->flip();
			else if (signedDistance(p_interior) == 0.f)
				throw CollinearityException();
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

		const Vec2& getOrigo() const
		{
			return origo;
		}

		HalfPlane2& rescale(float scale)
		{
			offset *= scale;
			origo = origo*scale;
			return *this;
		}

		HalfPlane2& shift(const Vec2& shift_vector)
		{
			offset = offset + shift_vector.dot(normal);
			origo = offset*normal;
			return *this;
		}

		HalfPlane2& flip()
		{
			normal = -1.f*normal;
			parallel = -1.f*parallel;
			offset *= -1.f;
			return *this;
		}

		// returns p's distance from the boundary (the sign is positive for points outside)
		float signedDistance(const Vec2& p) const
		{
			return p.dot(normal) - offset;
		}
		// returns the point where both halfplanes' boundaries intersect (beware parallel boundaries)
		Vec2 intersectBoundaries(const HalfPlane2& h) const
		{
			return origo + (h.offset - origo.dot(h.normal))/(parallel.dot(h.normal))*parallel;
		}

		class CollinearityException { };

	private:
		Vec2 normal; // the unit vector normal to the halfplane's boundary
		float offset; // the halfplane's offset from the origin in the direction of the normal
		Vec2 parallel; // the pair (normal, parallel) forms a right-handed orthonormal basis
		Vec2 origo; // orthogonal projection of the point (0,0) on the halfplane's boundary
	};
}

namespace AdditionalPrimitives2D
{
	class Circle
	{
	public:
		Circle() : center(), radius(0.f) { }
		Circle(const Geometry2D::Vec2& c, float r) : center(c), radius(r) { }
		Geometry2D::Vec2 center;
		float radius;
	};
}

#endif

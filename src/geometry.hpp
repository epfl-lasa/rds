#ifndef  GEOMETRY_HPP
#define  GEOMETRY_HPP

namespace Geometry2D
{
	// use this value to specify the range wherein the data and the results must lie
	// to be meaningful in your application.
	static const float range = 100.0; // (a large value smaller than infinity)

	// use this value to specify the resolution for the input data such that differences
	// below this value can be treated as zero.
	static const float resolution = 0.0001; // (a small value larger than zero)

	// returns true if numerator/denominator would lie outside [-range, range],
	// returns false otherwise.
	bool divisionIsOutsideRange(float numerator, float denominator);

	class Vec2
	{
	public:
		Vec2();
		Vec2(float x, float y);

		// computes the scalar product with another vector
		float dot(const Vec2& vector) const;

		// computes the euclidean norm
		float norm() const;

		float x, y;
	};

	Vec2 operator* (float, const Vec2&); // vector multiplication by a scalar
	Vec2 operator* (const Vec2&, float); // vector multiplication by a scalar
	Vec2 operator/ (const Vec2&, float); // vector division by a scalar
	Vec2 operator+ (const Vec2&, const Vec2&); // vector addition
	Vec2 operator- (const Vec2&, const Vec2&); // vector subtraction

	// computes the incircle of the triangle which the three points define
	// and returns the radius and assigns the incircle center coordinates to
	// the variable to which incircle_center is pointing if it is not the null pointer.
	float incircleRadius(const Vec2& corner_a, const Vec2& corner_b, const Vec2& corner_c,
		Vec2* incircle_center = 0);

	// defines a closed half-plane
	// ("closed" because the boundary line belongs to the half-plane)
	// ASSUMES THAT THE MEMBER normal IS NORMALIZED.
	class HalfPlane2
	{
	public:
		HalfPlane2();
		HalfPlane2(const Vec2& normal, float offset);

		// returns true if the point belongs to the half-plane or false otherwise
		bool contains(const Vec2& point_position_vector) const;

		// returns the point's orthogonal projection on the half-plane's boundary
		Vec2 boundaryProjection(const Vec2& point_position_vector) const;

		// returns the point where the two half-planes' boundaries intersect.
		// sets outside_range to true if any of the point's absolute coordinates
		// is larger than range (static member defined below), and to false otherwise.
		Vec2 boundaryIntersection(const HalfPlane2& h, bool* outside_range) const;

		Vec2 normal;
		float offset;
	};
}

#endif

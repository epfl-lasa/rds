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

	bool divisionIsInRange(float numerator, float denominator, float range_a, float range_b);

	class Vec2
	{
	public:
		Vec2() : x(0.0), y(0.0) { }
		Vec2(float x, float y) : x(x), y(y) { }
		// computes the scalar product with another vector
		float dot(const Vec2& v) const { return x*v.x + y*v.y; }
		// computes the euclidean norm
		float norm() const;
		// returns a copy rotated by 90 degrees in the counter-clockwise sense
		Vec2 orth() const { return Vec2(-y, x); }

		float x, y;
	};

	inline Vec2 operator* (float s, const Vec2& v) { return Vec2(v.x*s, v.y*s); } // vector multiplication by a scalar
	inline Vec2 operator* (const Vec2& v, float s) { return Vec2(v.x*s, v.y*s); }; // vector multiplication by a scalar
	inline Vec2 operator/ (const Vec2& v, float s) { return Vec2(v.x/s, v.y/s); }; // vector division by a scalar
	inline Vec2 operator+ (const Vec2& v1, const Vec2& v2) { return Vec2(v1.x + v2.x, v1.y + v2.y); }; // vector addition
	inline Vec2 operator- (const Vec2& v1, const Vec2& v2) { return Vec2(v1.x - v2.x, v1.y - v2.y); }; // vector subtraction
	inline Vec2 operator- (const Vec2& v) { return Vec2(-v.x,-v.y); }; // vector reversal

	// computes the incircle of the triangle which the three points define
	// and returns the radius and assigns the incircle center coordinates to
	// the variable to which incircle_center is pointing if it is not the null pointer.
	float incircleRadius(const Vec2& corner_a, const Vec2& corner_b, const Vec2& corner_c,
		Vec2* incircle_center = 0);

	class Circle2
	{
	public:
		Circle2() : center(Vec2()), radius(0.0) { }
		Circle2(Vec2 c, float r) : center(c), radius(r) { }
		Vec2 center;
		float radius;
	};

	Circle2 computeIncircle(const Vec2& corner_a, const Vec2& corner_b, const Vec2& corner_c);

	enum class AABBLineIntersection : int
	{
		x_lower_x_upper, // common cases
		y_lower_y_upper,
		x_lower_y_lower,
		x_lower_y_upper,
		x_upper_y_lower,
		x_upper_y_upper,
		no_intersection,
		/*line_x_upper, // edge cases
		line_x_lower,
		line_y_upper,
		line_y_lower,
		corner_x_lower_y_lower, // corner cases
		corner_x_lower_y_upper,
		corner_x_upper_y_lower,
		corner_x_upper_y_upper,*/
	};

	class AxisAlignedBoundingBox2;

	// defines a closed half-plane
	// ("closed" because the boundary line belongs to the half-plane)
	// ASSUMES THAT THE MEMBER normal IS NORMALIZED.
	class HalfPlane2
	{
	public:
		HalfPlane2() : normal(), offset(0.0) { }
		HalfPlane2(const Vec2& normal, float offset) : normal(normal), offset(offset) { }

		const HalfPlane2& flip();

		// returns true if the point p belongs to the half-plane or false otherwise
		bool contains(const Vec2& p) const { return (p.dot(normal) <= offset); }

		// returns the point p's orthogonal projection on the half-plane's boundary
		Vec2 boundaryProjection(const Vec2& p) const { return p + (offset - p.dot(normal))*normal; }

		// returns the point where the two half-planes' boundaries intersect.
		// sets outside_range to true if any of the point's absolute coordinates
		// is larger than range (constant defined at the top), and to false otherwise.
		Vec2 boundaryIntersection(const HalfPlane2& h, bool* outside_range) const;

		// returns the point where the two half-planes' boundaries intersect.
		// Caution: it does not check if the problem is well-posed (so it might create
		// INF or NAN or crash for boundaries that are close to parallel).
		Vec2 boundaryIntersection(const HalfPlane2& h) const { return offset*normal +
			(h.offset - offset*normal.dot(h.normal))/normal.orth().dot(h.normal)*normal.orth();
		}
		// intersects the two half-planes' boundaries.
		// returns the intersection's line coordinate along this line, starting from
		// the specified base point with the positive direction obtained by rotating
		// this half-plane's normal by 90 degrees counter-clockwise.
		// Caution: it does not check if the problem is well-posed (so it might create
		// INF or NAN or crash for boundaries that are close to parallel).
		float boundaryIntersection(const HalfPlane2& h, const Vec2& base_point) const {
			return (h.offset - base_point.dot(h.normal))/normal.orth().dot(h.normal);
		}
		// checks the intersection of this boundary with the other half-plane's boundary.
		// Returns true if the intersection's line coordinate along this boundary (for the specified base
		// point and the usual directional convention) is within the specified range,
		// and false otherwise.
		bool rangeCheckBoundaryIntersection(const HalfPlane2& h, const Vec2& base_point,
			float range_a, float range_b) const { 
				return divisionIsInRange(h.offset - base_point.dot(h.normal),
					normal.orth().dot(h.normal), range_a, range_b);
		}
		// Checks if the intersection between this half-plane's boundary and the given other 
		// half-plane's boundary lies within the specified axis-aligned bounding box.
		bool checkIfBoundaryIntersectionIsInBoundingBox(const HalfPlane2& other_half_plane,
			const AxisAlignedBoundingBox2& aabb) const;
/*
		// Shifts the point on this boundary to where both half-plane boundaries intersect.
		// Caution: it does not check if the problem is well-posed (so it might create
		// INF or NAN or crash for boundaries that are close to parallel). It returns the sign of
		// the shift, which is positive in the direction obtained by rotating this half-plane's normal 
		// by 90 degrees counter-clockwise.
		int shiftPointOnThisBoundaryToIntersection(const HalfPlane2& other_half_plane,
			Vec2* point_on_this_half_plane_boundary) const;
*/
		// Shifts the point along this boundary in the direction given by the counter-clockwise rotated normal
		void shiftPointAlongBoundary(float shift, Vec2* p) const { *p = *p + shift*normal.orth(); }

/*
		// Checks if the intersection between this half-plane's boundary and the given other 
		// half-plane's boundary lies within the specified axis-aligned bounding box:
		// if yes,
		// 		intersects the two boundaries, and
		// 		returns the intersection point's coordinate along this boundary, with the 
		// 		coordinate starting from the specified base point and the increasing direction 
		//		obtained by rotating this half-plane's normal by 90 degrees counter-clockwise,
		// if no,
		//		intersects this boundary with the infinite line defined by that side of the
		// 		axis-aligned bounding box whose outwards normal's direction is closest to
		// 		this half-plane's boundary direction ... in the direction where the intersection
		//		with the other constraint lies (but is too far away)... but what if half-planes are parallel?
		float boundedBoundaryIntersection(const Vec2& base_point_on_this_boundary,
			const HalfPlane2& other_half_plane, const AxisAlignedBoundingBox2& aabb);
*/
		Vec2 normal;
		float offset;
	};

	//struct LineSegment2
	//{
	//	Vec2 base_point;
	//	Vec2 direction;
	//	float 
	//}

	class AxisAlignedBoundingBox2
	{
	public:
		AxisAlignedBoundingBox2(float x_lower, float x_upper, float y_lower, float y_upper);
		const float x_lower, x_upper, y_lower, y_upper;
		const HalfPlane2 h_x_lower, h_x_upper, h_y_lower, h_y_upper;
		const Vec2 c_x_lower_y_lower, c_x_upper_y_lower, c_x_lower_y_upper, c_x_upper_y_upper;
		AABBLineIntersection intersectWithHalfPlaneBoundary(const HalfPlane2& h) const;
	};
}

#endif

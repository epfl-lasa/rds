#ifndef DISTANCE_MINIMIZER_HPP
#define DISTANCE_MINIMIZER_HPP

#include "geometry.hpp"

#include <vector>

namespace Geometry2D
{
	class DistanceMinimizer
	{
	public:
		// Finds the feasible point with minimum distance to the nominal point 
		// or if there is no feasible point, reports infeasibility.
		// It requires to specify an axis-aligned bounding box which contains
		// the feasible region if it exists.
		static void findPointClosestToNominal(const Vec2& nominal_point,
			const AxisAlignedBoundingBox2& aabb,
			const std::vector<HalfPlane2>& constraints,
			Vec2* solution_point, bool* feasible);
	};
}
#endif
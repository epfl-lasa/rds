#ifndef DISTANCE_MINIMIZER_HPP
#define DISTANCE_MINIMIZER_HPP

#include "geometry.hpp"

#include <vector>

namespace Geometry2D
{
	class DistanceMinimizer
	{
	public:

	//private:

		// Function minimizeDistanceToGoalOverConvexPolygon

		// Arguments:
		// constraints					...
		// goal							...
		// max_feasible_region_size		... upper bound for the diagonal of the bounding box containing the
		//									convex polygon and the nominal point
		// min_feasible_region_size		... lower bound for the diameter of the convex polygon's incircle
		// solution						...

		// Returns an error code with the following meaning:
		// 0	no error
		// 1	the specified value for min_feasible_region_size is too small
		//(2)	[not used anymore] the solver stumbled over a proof that the given value for min_feasible_region_size is incorrect
		// 3	there is no feasible region

		// it is still the caller's responsibility to put as the first four constraints
		// the box constraints having the given diagonal, otherwise the solution may be incorrect...
		// I am not sure about that, though (if later iterations will correct for skipped intersections...)

		static int minimizeDistanceToGoalOverConvexPolygon(const std::vector<HalfPlane2>& constraints,
			const Vec2& goal, float max_feasible_region_size, float min_feasible_region_size, Vec2* solution,
			float* bound_inaccuracy_due_to_skipping);

		static int incrementalMinimization(std::vector<HalfPlane2>::size_type iteration,
			const std::vector<HalfPlane2>& constraints, const Vec2& goal, float skipping_threshold_inner_product,
			Vec2* solution);

		static const float skipping_threshold_angle;
	};
}
#endif
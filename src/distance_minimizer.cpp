#include "distance_minimizer.hpp"

#include <cmath>

namespace Geometry2D
{
	// assume that the computer can still intersect reliably lines with such a small angle between them
	const float DistanceMinimizer::skipping_threshold_angle = (2.f*3.141592653589793f/360.f)*0.01f;

	int DistanceMinimizer::minimizeDistanceToGoalOverConvexPolygon(const std::vector<HalfPlane2>& constraints,
		const Vec2& goal, float max_feasible_region_size, float min_feasible_region_size, Vec2* solution,
		float* bound_inaccuracy_due_to_skipping)
	{
		float bb_a = std::sqrt(max_feasible_region_size*max_feasible_region_size -
			min_feasible_region_size*min_feasible_region_size);
		float bb_b = min_feasible_region_size;
		float inc_r = min_feasible_region_size/2.f;
		float min_feasible_sector_angle = 2.f*std::asin(inc_r/
			std::sqrt((bb_a - inc_r)*(bb_a - inc_r) + (bb_b - inc_r)*(bb_b - inc_r)));

		if (skipping_threshold_angle > min_feasible_sector_angle)
			return 1;

		*bound_inaccuracy_due_to_skipping = std::tan(skipping_threshold_angle)*max_feasible_region_size;

		float skipping_threshold_inner_product = std::cos(skipping_threshold_angle);
		//float aborting_threshold_inner_product = std::cos(3.141592653589793f - min_feasible_sector_angle);

		// initialize the incremental solution for the case that there are no constraints (0-th increment)
		*solution = goal;
		// call the first increment, which returns the result of the second increment, and so on.
		return incrementalMinimization(1, constraints, goal, skipping_threshold_inner_product,
			solution);
	}

// check projection distance before intersecting and return if the promise is false
// not necessary if the first four constraints have the specified bounding box diagonal 
// they must be given...!

	int DistanceMinimizer::incrementalMinimization(std::vector<HalfPlane2>::size_type iteration,
		const std::vector<HalfPlane2>& constraints, const Vec2& goal, float skipping_threshold_inner_product,
		Vec2* solution)
	{
		const HalfPlane2& hi = constraints[iteration - 1];
		if (!hi.contains(*solution))
		{
			Vec2 projected_goal = hi.projectOnBoundary(goal);
			Vec2 solution_candidate = projected_goal;
			float shift = 0.f;
			for (std::vector<HalfPlane2>::size_type j = 0; j < iteration - 1; j++)
			{
				const HalfPlane2& hj = constraints[j];
				if (!hj.contains(solution_candidate))
				{
					float normals_i_j_inner_product = hj.getNormal()*hi.getNormal();
					if ((normals_i_j_inner_product > skipping_threshold_inner_product) || // (*) SKIPPING CONDITIONS
						(normals_i_j_inner_product < -skipping_threshold_inner_product)) // (**)
						continue;
					//if (normals_i_j_inner_product < aborting_threshold_inner_product)
					//	return 2;
					float new_shift = hi.getParallelShiftToAnotherBoundary(projected_goal, hj);
					if ((shift > 0.f && new_shift < shift) || (shift < 0.f && new_shift > shift))
						return 3;
					solution_candidate = hi.applyParallelShift(new_shift, projected_goal);
					shift = new_shift;
				}
			}
			*solution = solution_candidate;
		}
		if (iteration == constraints.size())
			return 0;		
		return incrementalMinimization(iteration + 1, constraints, goal, skipping_threshold_inner_product,
			solution);
	}
}

/* 
Analysis of the modification to use the skipping conditions (*), (**).

	The analysis aims to proof that the skipping conditions do not alter the solution (significantly).

	Assume that 
	(i)		the first four constraints in the constraints vector create a bounding box
			which contains the goal point
	(ii)	the diagonal of that bounding box is shorter than max_feasible_region_size
	(iii)	there is a feasible region in that bounding box whose incircle has a
			radius larger than min_feasible_region_size.

	Propositions
	(A)		Whenever condition (*) evaluates to true and the intersection would lie in the bounding box,
			the resulting shift would be smaller than tan(skipping_threshold_angle)*max_feasible_region_size.
	(B)		Condition (**) never evaluates to true when the intersection point would lie in the bounding box.
	(C)		In each iteration, the intersections that follow after re-introducing the first three constraints
			lie in the bounding box.
	(D)		When re-introducing the first three constraints in an iteration, intersections that do not lie in
			the bounding box can be ignored without affecting the final outcome of the iteration.
	(E)		It follows that whenever conditions (*), (**) lead to skipping an intersection, there is no effect
			or a negligible effect on the outcome of the iteration given that the values of
			skipping_threshold_angle and max_feasible_region_size are small enough.

	The proofs for the propositions will be given elsewhere (sketches are given in docs/iqp_analysis.svg).
*/
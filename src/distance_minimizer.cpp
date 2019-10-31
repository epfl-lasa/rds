#include "distance_minimizer.hpp"

#include <cmath>

//#include <iostream>

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
					float normals_i_j_inner_product = hj.getNormal().dot(hi.getNormal());
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

	// initialize the static members
	const std::vector<HalfPlane2> DistanceMinimizer::unitSquareConstraints {HalfPlane2(Vec2(1,0), 0.5), 
		HalfPlane2(Vec2(-1,0), 0.5), HalfPlane2(Vec2(0,1), 0.5), HalfPlane2(Vec2(0,-1), 0.5)};

	const float DistanceMinimizer::boundingCircleRadius = std::sqrt(2.f)/2.f + 0.01f;

	const float DistanceMinimizer::distanceTolerance = 0.00001f;

	float criticalCuttingAngle = std::asin(DistanceMinimizer::distanceTolerance/2.f/
			DistanceMinimizer::boundingCircleRadius);
	const float DistanceMinimizer::cuttingAngleCosineThreshold = std::cos(criticalCuttingAngle);

	Vec2 DistanceMinimizer::IncrementalDistanceMinimization(const std::vector<HalfPlane2>& feasible_halfplanes)
	{
		Vec2 p_s(0.f,0.f);
		for (HalfplaneIndex j = 0; j != feasible_halfplanes.size(); ++j)
			p_s = DistanceMinimizationIncrementation(j, feasible_halfplanes, p_s);
		return p_s;
	}

	Vec2 DistanceMinimizer::DistanceMinimizationIncrementation(HalfplaneIndex this_index,
			const std::vector<HalfPlane2>& feasible_halfplanes, const Vec2& prior_solution)
	{
		Vec2 p_s = prior_solution; // initialize the solution as the prior solution
		const HalfPlane2& H_new = feasible_halfplanes[this_index]; // just for convenience
		if (H_new.signedDistance(p_s) <= distanceTolerance) // change nothing in that case
			return p_s;
		else if (H_new.getOffset() > boundingCircleRadius) // H_new and the central unit square are disjoint
			throw InfeasibilityException(1);
		else // search on H_new's boundary for the solution
		{
			p_s = H_new.getOrigo(); // orthogonal projection of the goal (0,0) on H's boundary
			Vec2 p_1 = p_s + H_new.getParallel()*2.f*boundingCircleRadius;
			Vec2 p_2 = p_s - H_new.getParallel()*2.f*boundingCircleRadius; // feasible segment's endpoints
			for (HalfplaneIndex j = 0; j != 4; ++j) // process the unit square constraints first (why not?)
			{
				//std::cout << "bb " << j << std::endl;
				processPriorHalfplane(H_new, unitSquareConstraints[j], &p_s, &p_1, &p_2);
			}
			for (HalfplaneIndex j = 0; j != this_index; ++j) // process the given prior halfplanes
			{
				//std::cout << "h " << j << std::endl;
				processPriorHalfplane(H_new, feasible_halfplanes[j], &p_s, &p_1, &p_2);
			}
			return p_s; // return the solution found on H_new's boundary (no infeasibility exception occurred)
		}
	}

	void DistanceMinimizer::processPriorHalfplane(const HalfPlane2& H_new, const HalfPlane2& H_prior, Vec2* p_s,
		Vec2* p_1, Vec2* p_2)
	{
		bool flat_angle(std::abs(H_prior.getNormal().dot(H_new.getNormal())) > cuttingAngleCosineThreshold);
		if (H_prior.signedDistance(*p_s) > distanceTolerance)
			*p_s = flat_angle ? throw InfeasibilityException(2) : H_new.intersectBoundaries(H_prior);

		if ((H_prior.signedDistance(*p_1) > distanceTolerance) &&
			(H_prior.signedDistance(*p_2) <= distanceTolerance))
			*p_1 = flat_angle ? throw InfeasibilityException(3) : H_new.intersectBoundaries(H_prior);
		else if ((H_prior.signedDistance(*p_1) <= distanceTolerance) &&
			(H_prior.signedDistance(*p_2) > distanceTolerance))
			*p_2 = flat_angle ? throw InfeasibilityException(4) : H_new.intersectBoundaries(H_prior);
		else if ((H_prior.signedDistance(*p_1) > distanceTolerance) &&
			(H_prior.signedDistance(*p_2) > distanceTolerance))
			throw InfeasibilityException(5);
	}
					/*if (flat_angle)
						throw InfeasibilityException();
					else
						p_s = H_new.intersectBoundaries();*/
}
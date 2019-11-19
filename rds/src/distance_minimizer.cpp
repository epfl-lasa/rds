#include "distance_minimizer.hpp"
#include <cmath>

namespace Geometry2D
{
	// initialize the static members
	const std::vector<HalfPlane2> DistanceMinimizer::unitSquareConstraints {HalfPlane2(Vec2(1,0), 0.5), 
		HalfPlane2(Vec2(-1,0), 0.5), HalfPlane2(Vec2(0,1), 0.5), HalfPlane2(Vec2(0,-1), 0.5)};

	const float DistanceMinimizer::boundingCircleRadius = std::sqrt(2.f)/2.f + 0.01f;

	const float DistanceMinimizer::distanceTolerance = 0.00001f;

	const float DistanceMinimizer::criticalCuttingAngle = std::asin(DistanceMinimizer::distanceTolerance/2.f/
			DistanceMinimizer::boundingCircleRadius);

	const float DistanceMinimizer::cuttingAngleCosineThreshold = std::cos(DistanceMinimizer::criticalCuttingAngle);

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
		else if (H_new.getOffset() < -boundingCircleRadius) // H_new and the central unit square are disjoint
			throw InfeasibilityException(1); // actually the subsequent checks would also find this
		else // search on H_new's boundary for the solution
		{
			p_s = H_new.getOrigo(); // orthogonal projection of the goal (0,0) on H's boundary
			Vec2 p_1 = p_s + H_new.getParallel()*2.f*boundingCircleRadius;
			Vec2 p_2 = p_s - H_new.getParallel()*2.f*boundingCircleRadius; // feasible segment's endpoints
			for (HalfplaneIndex j = 0; j != 4; ++j) // process the unit square constraints first (why not?)
				processPriorHalfplane(H_new, unitSquareConstraints[j], &p_s, &p_1, &p_2);
			for (HalfplaneIndex j = 0; j != this_index; ++j) // process the given prior halfplanes
				processPriorHalfplane(H_new, feasible_halfplanes[j], &p_s, &p_1, &p_2);
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
}
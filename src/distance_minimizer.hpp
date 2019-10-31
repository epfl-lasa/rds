#ifndef DISTANCE_MINIMIZER_HPP
#define DISTANCE_MINIMIZER_HPP

#include "geometry.hpp"
#include <vector>

namespace Geometry2D
{
	class DistanceMinimizer
	{
	public:
		// Returns the point with minimum Euclidean norm in the intersection of the centred unit square
		// and the given feasible halfplanes. If this intersection is empty, the function throws an
		// infeasibility exception. Note that the centred unit square's corners are at (+/-0.5, +/-0.5).
		static Vec2 IncrementalDistanceMinimization(const std::vector<HalfPlane2>& feasible_halfplanes);

	private:
		typedef std::vector<HalfPlane2>::size_type HalfplaneIndex;

		static Vec2 DistanceMinimizationIncrementation(HalfplaneIndex this_index,
			const std::vector<HalfPlane2>& feasible_halfplanes, const Vec2& prior_solution);

		static void processPriorHalfplane(const HalfPlane2& H_new, const HalfPlane2& H_prior, Vec2* p_s,
			Vec2* p_1, Vec2* p_2);

	public:
		class InfeasibilityException
		{
		public:
			InfeasibilityException(int code) : code(code) { }
			int code;
		};

		static const std::vector<HalfPlane2> unitSquareConstraints;
		static const float boundingCircleRadius;
		static const float distanceTolerance;
		static const float criticalCuttingAngle;
		static const float cuttingAngleCosineThreshold;
	};
}
#endif
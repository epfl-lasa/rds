#include "distance_minimizer.hpp"

namespace Geometry2D
{
	void DistanceMinimizer::findPointClosestToNominal(const Vec2& nominal_point,
			const AxisAlignedBoundingBox2& aabb,
			const std::vector<HalfPlane2>& constraints,
			Vec2* solution_point, bool* feasible)
	{
		// for convenience
		Vec2 & p = *solution_point;
		// initialize at the solution as the nominal point
		p = nominal_point;
		// add constraints one by one
		for (std::vector<HalfPlane2>::size_type i = 0; i < constraints.size(); i++)
		{
			if (!constraints[i].contains(p))
			{
				p = constraints[i].boundaryProjection(nominal_point);
				for (std::vector<HalfPlane2>::size_type j = 0; j < i; j++)
				{
					int sign = 0;
					if (!constraints[j].contains(p))
					{
						if (constraints[i].checkIfBoundaryIntersectionIsInBoundingBox(constraints[j],
							aabb))
						{
							int new_sign = constraints[i].shiftPointOnThisBoundaryToIntersection(
								constraints[j], &p);
							if (((new_sign > 0) && (sign < 0)) || ((new_sign < 0) && (sign > 0)))
							{
								*feasible = false;
								return;	
							}
							else if (new_sign != 0)
								sign = new_sign;
						}
						else
						{
							*feasible = false;
							return;
						}
					}
				}
			}
		}
		*feasible = true;
	}
}
#include "geometry.hpp"
#include "distance_minimizer.hpp"
#include "gui.hpp"

#include <iostream>
#include <vector>

int main()
{
	// set up and solve a distance minimization problem
	std::vector<Geometry2D::HalfPlane2> constraints;
	constraints.push_back(Geometry2D::HalfPlane2(Geometry2D::Vec2(1.0, 0.0), 1.5));
	constraints.push_back(Geometry2D::HalfPlane2(Geometry2D::Vec2(0.0, 1.0), 0.5));
	Geometry2D::Vec2 v(-0.5, -1.0);
	constraints.push_back(Geometry2D::HalfPlane2(v/v.norm(), 0.4));
	constraints.push_back(Geometry2D::HalfPlane2(Geometry2D::Vec2(0.0, -1.0), 0.05));
	constraints.push_back(Geometry2D::HalfPlane2(Geometry2D::Vec2(0.0, -1.0), -0.02));
	//Geometry2D::Vec2 v2(1.0, -0.5);
	//constraints.push_back(Geometry2D::HalfPlane2(v2/v2.norm(), -0.95));

	// define a bounding box such that the feasible must be contained in the interior with no
	// common point with the border of the bounding box.
	Geometry2D::AxisAlignedBoundingBox2 aabb(-3.0, 3.0, -3.0, 3.0);

	Geometry2D::Vec2 nominal_point(-1.0, -1.0);
	
	Geometry2D::Vec2 solution_point;
	bool feasible;
	Geometry2D::DistanceMinimizer::findPointClosestToNominal(nominal_point,
		aabb, constraints, &solution_point, &feasible);

	if (feasible)
		std::cout << "The solution is feasible." << std::endl;
	else
		std::cout << "There is no feasible point." << std::endl;

	// draw the problem and the solution in a GUI
	std::vector<Geometry2D::Vec2> points;
	points.push_back(solution_point);
	points.push_back(nominal_point);
	GUI gui(0.0, constraints, points, std::vector<Geometry2D::Circle2>(0));

	return 0;
}
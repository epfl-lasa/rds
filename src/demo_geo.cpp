#include "geometry.hpp"
#include "distance_minimizer.hpp"
#include "gui.hpp"

#include <iostream>
#include <vector>
#include <cmath>

int main()
{
	// set up and solve a distance minimization problem
	std::vector<Geometry2D::HalfPlane2> constraints;
	constraints.push_back(Geometry2D::HalfPlane2(Geometry2D::Vec2(1.0, 0.0), 1.5));
	constraints.push_back(Geometry2D::HalfPlane2(Geometry2D::Vec2(0.0, 1.0), 0.5));
	constraints.push_back(Geometry2D::HalfPlane2(Geometry2D::Vec2(-0.5, -1.0), 0.4));
	constraints.push_back(Geometry2D::HalfPlane2(Geometry2D::Vec2(0.0, -1.0), 0.05));
	constraints.push_back(Geometry2D::HalfPlane2(Geometry2D::Vec2(0.0, -1.0), -0.02));
	constraints.push_back(Geometry2D::HalfPlane2(Geometry2D::Vec2(-1.5, -0.5), -0.02));
	//Geometry2D::Vec2 v2(1.0, -0.5);
	//constraints.push_back(Geometry2D::HalfPlane2(v2/v2.norm(), -0.95));

	Geometry2D::Vec2 nominal_point(-1.0, -1.0);

	Geometry2D::Vec2 solution_point;
	float bound_inaccuracy_due_to_skipping;

	int error = Geometry2D::DistanceMinimizer::minimizeDistanceToGoalOverConvexPolygon(constraints,
			nominal_point, 10.0, 0.01, &solution_point, &bound_inaccuracy_due_to_skipping);

	if (error)
		std::cout << "Error code: " << error << std::endl;
	else
	{
		std::cout << "Found the feasible solution without encountering any errors." << std::endl;
		std::cout << "bound_inaccuracy_due_to_skipping: " << bound_inaccuracy_due_to_skipping << std::endl;
	}

	// draw the problem and the solution in a GUI
	std::vector<Geometry2D::Vec2> points;
	points.push_back(solution_point);
	points.push_back(nominal_point);
	GUI gui(0.0, constraints, points);

	return 0;
}
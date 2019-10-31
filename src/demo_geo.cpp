#include "geometry.hpp"
#include "distance_minimizer.hpp"
#include "gui.hpp"

#include <iostream>
#include <vector>
#include <cmath>

int main()
{
	Geometry2D::Vec2 nominal_point(-0.1, -0.1);

	// set up and solve a distance minimization problem
	std::vector<Geometry2D::HalfPlane2> constraints;
	constraints.push_back(Geometry2D::HalfPlane2(Geometry2D::Vec2(1.0, 0.0), 1.5/10.f-nominal_point.dot(Geometry2D::Vec2(1.0, 0.0).normalized())));
	constraints.push_back(Geometry2D::HalfPlane2(Geometry2D::Vec2(0.0, 1.0), 0.5/10.f-nominal_point.dot(Geometry2D::Vec2(0.0, 1.0).normalized())));
	constraints.push_back(Geometry2D::HalfPlane2(Geometry2D::Vec2(-0.5, -1.0), 0.4/10.f-nominal_point.dot(Geometry2D::Vec2(-0.5, -1.0).normalized())));
	constraints.push_back(Geometry2D::HalfPlane2(Geometry2D::Vec2(0.0, -1.0), 0.05/10.f-nominal_point.dot(Geometry2D::Vec2(0.0, -1.0).normalized())));
	constraints.push_back(Geometry2D::HalfPlane2(Geometry2D::Vec2(0.0, -1.0), -0.02/10.f-nominal_point.dot(Geometry2D::Vec2(0.0, -1.0).normalized())));
	constraints.push_back(Geometry2D::HalfPlane2(Geometry2D::Vec2(0.0, 1.0), 0.12/10.f-nominal_point.dot(Geometry2D::Vec2(0.0, 1.0).normalized())));
	constraints.push_back(Geometry2D::HalfPlane2(Geometry2D::Vec2(-1.5, -0.5), -0.02/10.f-nominal_point.dot(Geometry2D::Vec2(-1.5, -0.5).normalized())));
	constraints.push_back(Geometry2D::HalfPlane2(Geometry2D::Vec2(1.0, 0.0), -0.1/10.f-nominal_point.dot(Geometry2D::Vec2(1.0, 0.0).normalized())));
	//Geometry2D::Vec2 v2(1.0, -0.5);
	//constraints.push_back(Geometry2D::HalfPlane2(v2/v2.norm(), -0.95));

	nominal_point = Geometry2D::Vec2(0.f, 0.f);

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
		std::cout << "Solution point: (" << solution_point.x << ", " << solution_point.y << ")"<< std::endl;
	}

	// draw the problem and the solution in a GUI
	std::vector<Geometry2D::Vec2> points;
	points.push_back(solution_point);
	points.push_back(nominal_point);
	GUI gui(0.0, constraints, points);

	try
	{
		Geometry2D::Vec2 p_s = Geometry2D::DistanceMinimizer::IncrementalDistanceMinimization(constraints);
		points[0] = p_s;
		std::cout << "Solution: (" << p_s.x << ", " << p_s.y << ")"<< std::endl;
		GUI gui_2(0.0, constraints, points);
	}
	catch (Geometry2D::DistanceMinimizer::InfeasibilityException e)
	{
		std::cout << "IncrementalDistanceMinimization: infeasible, code " << e.code << std::endl;
	}

	return 0;
}
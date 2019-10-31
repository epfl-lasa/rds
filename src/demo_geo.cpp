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
	//constraints.push_back(Geometry2D::HalfPlane2(Geometry2D::Vec2(1.0, 0.0), -0.1/10.f-nominal_point.dot(Geometry2D::Vec2(1.0, 0.0).normalized())));
	//Geometry2D::Vec2 v2(1.0, -0.5);
	//constraints.push_back(Geometry2D::HalfPlane2(v2/v2.norm(), -0.95));

	nominal_point = Geometry2D::Vec2(0.f, 0.f);

	try
	{
		Geometry2D::Vec2 p_s = Geometry2D::DistanceMinimizer::IncrementalDistanceMinimization(constraints);
		std::cout << "Solution: (" << p_s.x << ", " << p_s.y << ")"<< std::endl;
		// draw the problem and the solution in a GUI
		std::vector<Geometry2D::Vec2> points;
		points.push_back(p_s);
		points.push_back(nominal_point);

		GUI gui("Minimum Norm Point", 0.5);
		gui.halfplanes = &constraints;
		gui.points = &points;
		gui.blockingShowUntilClosed();
	}
	catch (Geometry2D::DistanceMinimizer::InfeasibilityException e)
	{
		std::cout << "IncrementalDistanceMinimization: infeasible, code " << e.code << std::endl;
	}

	return 0;
}
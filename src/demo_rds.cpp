#include "rds_wrap.hpp"
#include "gui.hpp"
#include "differential_drive_kinematics.hpp"
#include "collision_point.hpp"
#include "geometry.hpp"

//#include <iostream>
#include <vector>
//#include <cmath>
//#include <chrono>

using RDS::VelocityCommand;
using RDS::VelocityCommandBoxLimits;
using RDS::VelocityCommandHexagonLimits;
using RDS:: CollisionPoint;
using Geometry2D::Vec2;

int main()
{
	VelocityCommand nominal_command(0.75f, 0.5f);

	VelocityCommandBoxLimits box_limits;
	box_limits.min_linear = 0.5f;
	box_limits.min_angular = 0.25f;
	box_limits.max_linear = 1.5f;
	box_limits.max_angular = 0.75f;

	VelocityCommandHexagonLimits hexagon_limits;
	hexagon_limits.min_linear = -0.75f;
	hexagon_limits.max_linear = 1.75f;
	hexagon_limits.absolute_angular_at_min_linear = 0.f;
	hexagon_limits.absolute_angular_at_max_linear = 0.f;
	hexagon_limits.absolute_angular_at_zero_linear = 1.f;

	std::vector<CollisionPoint> collision_points;
	collision_points.push_back(CollisionPoint(Vec2(1.f, 1.f), Vec2(0.7f, 0.7f), Vec2(0.f, 0.f)));

	float y_coordinate_of_reference_point_for_command_limits = 0.5f;
	float weight_scaling_of_reference_point_for_command_limits = 1.f;
	float tau = 2.f;
	float delta = 0.1f;
	float clearance_from_axle_of_final_reference_point = 0.15f;

	RDSWrap rds_wrap(nominal_command,
		box_limits,
		hexagon_limits,
		collision_points,
		y_coordinate_of_reference_point_for_command_limits,
		weight_scaling_of_reference_point_for_command_limits,
		tau,
		delta,
		clearance_from_axle_of_final_reference_point);

	GUI gui("Minimum Norm Point", 1.f);
	std::vector<Vec2> points;
	points.push_back(rds_wrap.getScaledShiftedSolution());
	points.push_back(Vec2(0.f, 0.f));
	gui.halfplanes = &rds_wrap.getScaledShiftedConstraints();
	gui.points = &points;
	Window::sdlColor green;
	green.r = green.b = 0;
	green.g = 255;
	gui.points_colors.push_back(green);
	gui.points_colors.push_back(green);
	gui.blockingShowUntilClosed();
}
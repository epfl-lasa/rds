#include "rds_wrap.hpp"
#include "gui.hpp"
#include "differential_drive_kinematics.hpp"
#include "collision_point.hpp"
#include "geometry.hpp"

//#include <iostream>
#include <vector>
//#include <cmath>
#include <chrono>
#include <thread>

using RDS::VelocityCommand;
using RDS::VelocityCommandBoxLimits;
using RDS::VelocityCommandHexagonLimits;
using RDS:: CollisionPoint;
using Geometry2D::Vec2;
using Geometry2D::HalfPlane2;
using AdditionalPrimitives2D::Arrow;

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

	Window::sdlColor green;
	green.r = green.b = 0;
	green.g = 255;
	Window::sdlColor blue;
	blue.r = blue.g = 0;
	blue.b = 255;
	Window::sdlColor red;
	red.g = red.b = 0;
	red.r = 255;
	Window::sdlColor magenta;
	magenta.r = magenta.b = 255;
	magenta.g = 0;
	Window::sdlColor cyan;
	cyan.g = cyan.b = 255;
	cyan.r = 0;
	Window::sdlColor yellow;
	yellow.r = 255;
	yellow.g = 215;
	yellow.b = 0;
	Window::sdlColor orange;
	orange.r = 255;
	orange.g = 127;
	orange.b = 80;

	GUI gui_solver("Minimum Norm Point", 1.f);
	std::vector<Vec2> points;
	points.push_back(rds_wrap.getScaledShiftedSolution());
	points.push_back(Vec2(0.f, 0.f));
	gui_solver.points = &points;
	gui_solver.points_colors.push_back(green);
	gui_solver.points_colors.push_back(blue);
	gui_solver.halfplanes = &rds_wrap.getScaledShiftedConstraints();
	//gui_solver.blockingShowUntilClosed();

	GUI gui_command("Linear and Angular Velocity Commands", 2.f);
	float normalization_linear = hexagon_limits.max_linear;
	float normalization_angular = hexagon_limits.absolute_angular_at_zero_linear;

	std::vector<Arrow> command_arrows;
	Vec2 head;
	head = Vec2(rds_wrap.getCommandSolution().linear/normalization_linear,
		rds_wrap.getCommandSolution().angular/normalization_angular);
	command_arrows.push_back(Arrow(head, Vec2(0.f, 0.f)));

	head = Vec2(nominal_command.linear/normalization_linear,
		nominal_command.angular/normalization_angular);
	command_arrows.push_back(Arrow(head, Vec2(0.f, 0.f)));

	gui_command.arrows_colors.push_back(green);
	gui_command.arrows_colors.push_back(blue);
	gui_command.arrows = &command_arrows;

	// add constraints transformed to command space
	std::vector<HalfPlane2> command_constraints;
	for (auto& h : rds_wrap.getReferencePointVelocityConstraints())
	{
		RDS::PointVelocityConstraint pvc(rds_wrap.getReferencePoint(), h);
		command_constraints.push_back(RDS::transformPointVelocityConstraintToCommandConstraint(pvc,
			normalization_linear, normalization_angular));
	}
	gui_command.halfplanes = &command_constraints;

	GUI gui_shapes("Points and Shapes", 4.f);
	std::vector<Vec2> gui_shapes_points;
	for (auto& cp : collision_points)
	{
		gui_shapes_points.push_back(cp.p);
		gui_shapes_points.push_back(cp.p + cp.p_to_q);
		gui_shapes.points_colors.push_back(yellow);
		gui_shapes.points_colors.push_back(orange);
	}
	gui_shapes_points.push_back(rds_wrap.getReferencePoint());
	gui_shapes.points_colors.push_back(cyan);
	gui_shapes_points.push_back(Vec2(0.f, y_coordinate_of_reference_point_for_command_limits));
	gui_shapes.points_colors.push_back(magenta);
	gui_shapes.points = &gui_shapes_points;

	while ((gui_solver.update() == 0) | (gui_command.update() == 0) | (gui_shapes.update() == 0))
		std::this_thread::sleep_for(std::chrono::milliseconds(50));
}
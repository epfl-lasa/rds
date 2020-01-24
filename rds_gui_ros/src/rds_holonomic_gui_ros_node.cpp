#include "rds_holonomic_gui_ros_node.hpp"

#include <rds/differential_drive_kinematics.hpp>

#include <chrono>
#include <thread>

#include <algorithm>
#include <random>

auto rng = std::default_random_engine {};

using Geometry2D::Vec2;
using Geometry2D::HalfPlane2;
using AdditionalPrimitives2D::Arrow;
using AdditionalPrimitives2D::Circle;
using RDS::VelocityCommand;
using RDS::PointVelocityConstraint;

void RDSHolonomicGUIROSNode::toGuiMessageCallback(const rds_network_ros::ToGui::ConstPtr& to_gui_msg)
{
	const rds_network_ros::ToGui& msg = *to_gui_msg;

	float linear_normalization = msg.limit_velocity_command_max_linear;
	float angular_normalization = msg.limit_velocity_command_abs_angular_at_zero_linear;

	Vec2 reference_point(msg.reference_point.x, msg.reference_point.y);

	Vec2 nominal_command(msg.nominal_command.angular, msg.nominal_command.linear);
	Vec2 corrected_command(msg.corrected_command.angular, msg.corrected_command.linear);

	command_space_arrows.resize(0);
	command_space_arrows.push_back(Arrow(nominal_command, Vec2()));
	command_space_arrows.push_back(Arrow(corrected_command, Vec2()));
	gui_command_space.arrows_colors.resize(0);
	gui_command_space.arrows_colors.push_back(color_nominal);
	gui_command_space.arrows_colors.push_back(color_corrected);

	command_space_halfplanes.resize(0);
	for (auto& h_msg : msg.reference_point_velocity_constraints)
	{
		PointVelocityConstraint pvc(reference_point, HalfPlane2(Vec2(h_msg.normal.x, h_msg.normal.y), h_msg.offset));
		command_space_halfplanes.push_back(RDS::transformPointVelocityConstraintToCommandConstraint(pvc,
			linear_normalization, angular_normalization));
	}
	std::shuffle(std::begin(command_space_halfplanes) + 10, std::end(command_space_halfplanes), rng);

	solver_space_points.resize(0);
	solver_space_points.push_back(Vec2());
	solver_space_points.push_back(Vec2(msg.solver_solution.x, msg.solver_solution.y));
	gui_solver_space.points_colors.resize(0);
	gui_solver_space.points_colors.push_back(color_nominal);
	gui_solver_space.points_colors.push_back(color_corrected);

	solver_space_halfplanes.resize(0);
	for (auto& h_msg : msg.solver_constraints)
		solver_space_halfplanes.push_back(HalfPlane2(Vec2(h_msg.normal.x, h_msg.normal.y), h_msg.offset));
	std::shuffle(std::begin(solver_space_halfplanes) + 10, std::end(solver_space_halfplanes), rng);

	work_space_points.resize(0);
	gui_work_space.points_colors.resize(0);

	Vec2 p_ref_com_lim(msg.reference_point_for_command_limits.x, msg.reference_point_for_command_limits.y);

	Vec2 v_nominal_p_ref_com_lim = nominal_command;
	Vec2 v_corrected_p_ref_com_lim = corrected_command;

	for (auto& p_msg : msg.collision_points_on_robot)
	{
		work_space_points.push_back(Vec2(p_msg.x, p_msg.y));
		gui_work_space.points_colors.push_back(orange);
	}
	for (auto& p_msg : msg.collision_points_on_obstacles)
	{
		work_space_points.push_back(Vec2(p_msg.x, p_msg.y));
		gui_work_space.points_colors.push_back(yellow);
	}

	//work_space_points.push_back(p_ref_com_lim);
	//gui_work_space.points_colors.push_back(magenta);

	//work_space_points.push_back(reference_point);
	//gui_work_space.points_colors.push_back(cyan);

	work_space_arrows.resize(0);
	gui_work_space.arrows_colors.resize(0);
	Vec2 reference_point_velocity_solution(msg.reference_point_velocity_solution.x, msg.reference_point_velocity_solution.y);
	Vec2 reference_point_nominal_velocity(msg.reference_point_nominal_velocity.x, msg.reference_point_nominal_velocity.y);
	work_space_arrows.push_back(Arrow(p_ref_com_lim + 1.4*v_nominal_p_ref_com_lim, p_ref_com_lim));
	work_space_arrows.push_back(Arrow(p_ref_com_lim + 1.4*v_corrected_p_ref_com_lim, p_ref_com_lim));
	gui_work_space.arrows_colors.push_back(color_nominal);
	gui_work_space.arrows_colors.push_back(color_corrected);

	work_space_circles.resize(0);
	for (auto& c_msg : msg.robot_shape)
		work_space_circles.push_back(Circle(Vec2(c_msg.center.x, c_msg.center.y), c_msg.radius));
}

RDSHolonomicGUIROSNode::RDSHolonomicGUIROSNode(ros::NodeHandle* n)
	: gui_command_space("Command Space", 2.f)
	, gui_solver_space("Solver Space", 1.f)
	, gui_work_space("Work Space", 4.f)
	, to_gui_subscriber(n->subscribe<rds_network_ros::ToGui>("rds_to_gui", 1,
		&RDSHolonomicGUIROSNode::toGuiMessageCallback, this))
{
	//gui_solver_space.activateHalfplaneAreaRendering();
	gui_command_space.arrows = &command_space_arrows;
	gui_command_space.halfplanes = &command_space_halfplanes;

	gui_solver_space.points = &solver_space_points;
	gui_solver_space.halfplanes = &solver_space_halfplanes;

	gui_work_space.points = &work_space_points;
	gui_work_space.circles = &work_space_circles;
	gui_work_space.arrows = &work_space_arrows;

	green.r = green.b = 0;
	green.g = 255;
	blue.r = blue.g = 0;
	blue.b = 255;
	red.g = red.b = 0;
	red.r = 255;
	magenta.r = magenta.b = 255;
	magenta.g = 0;
	cyan.g = cyan.b = 255;
	cyan.r = 0;
	blue_fade.b = 255;
	blue_fade.r = 100;
	blue_fade.g = 100;
	yellow.r = 255;
	yellow.g = 215;
	yellow.b = 0;
	orange.r = 255;
	orange.g = 127;
	orange.b = 80;
	white.b = white.g = white.r = 255;
	purple.r = 155;
	purple.g = 70;
	purple.b = 255;
	color_nominal.r = 30;
	color_nominal.g = 180;
	color_nominal.b = 255;
	color_corrected.r = 0;
	color_corrected.g = 255;
	color_corrected.b = 0;

	std::chrono::milliseconds gui_cycle_time(50);
	std::chrono::high_resolution_clock::time_point t1, t2;
	t1 = std::chrono::high_resolution_clock::now();
	while (ros::ok() && (
		(gui_solver_space.update() == 0) |
		(gui_command_space.update() == 0) |
		(gui_work_space.update() == 0)))
	{
		ros::spinOnce();
		t2 = std::chrono::high_resolution_clock::now();
		std::this_thread::sleep_for(gui_cycle_time - std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1));
		ROS_INFO("Gui updated in %d milliseconds.", std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1));
		t1 = t2;
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "rds_gui_ros_node");
	ros::NodeHandle n;
	RDSHolonomicGUIROSNode rds_gui_ros_node(&n);
	return 0;
}
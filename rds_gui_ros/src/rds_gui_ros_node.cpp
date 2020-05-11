#include "rds_gui_ros_node.hpp"

#include <chrono>
#include <thread>

#include <algorithm>
#include <random>

using Geometry2D::Vec2;
using Geometry2D::HalfPlane2;
using AdditionalPrimitives2D::Arrow;
using AdditionalPrimitives2D::Circle;
using Geometry2D::Capsule;

void RDSGUIROSNode::toGuiMessageCallback(const rds_network_ros::ToGui::ConstPtr& to_gui_msg)
{
	const rds_network_ros::ToGui& msg = *to_gui_msg;

	Vec2 v_p_ref_nominal(msg.reference_point_nominal_velocity.x, msg.reference_point_nominal_velocity.y);
	Vec2 v_p_ref_solution(msg.reference_point_velocity_solution.x, msg.reference_point_velocity_solution.y);

	command_space_arrows.resize(0);
	command_space_arrows.push_back(Arrow(v_p_ref_nominal, Vec2()));
	command_space_arrows.push_back(Arrow(v_p_ref_solution, Vec2()));
	gui_command_space.arrows_colors.resize(0);
	gui_command_space.arrows_colors.push_back(color_nominal);
	gui_command_space.arrows_colors.push_back(color_corrected);

	command_space_halfplanes.resize(0);
	for (auto& h_msg : msg.reference_point_velocity_constraints)
		command_space_halfplanes.push_back(HalfPlane2(Vec2(h_msg.normal.x, h_msg.normal.y), h_msg.offset));

	work_space_points.resize(0);
	gui_work_space.points_colors.resize(0);

	for (auto& p_msg : msg.moving_objects)
	{
		work_space_points.push_back(Vec2(p_msg.x, p_msg.y));
		gui_work_space.points_colors.push_back(yellow);
	}

	work_space_arrows.resize(0);

	work_space_capsules.resize(0);
	work_space_capsules.push_back(Capsule(msg.robot_shape.radius,
		Vec2(msg.robot_shape.center_a.x, msg.robot_shape.center_a.y),
		Vec2(msg.robot_shape.center_b.x, msg.robot_shape.center_b.y)));
}

RDSGUIROSNode::RDSGUIROSNode(ros::NodeHandle* n)
	: gui_command_space("Command Space", 2.f)
	, gui_solver_space("Solver Space", 1.f)
	, gui_work_space("Work Space", 4.f)
	, to_gui_subscriber(n->subscribe<rds_network_ros::ToGui>("rds_to_gui", 1,
		&RDSGUIROSNode::toGuiMessageCallback, this))
{
	//gui_solver_space.activateHalfplaneAreaRendering();
	gui_command_space.arrows = &command_space_arrows;
	gui_command_space.halfplanes = &command_space_halfplanes;

	gui_solver_space.points = &solver_space_points;
	gui_solver_space.halfplanes = &solver_space_halfplanes;

	gui_work_space.points = &work_space_points;
	gui_work_space.arrows = &work_space_arrows;
	gui_work_space.capsules = &work_space_capsules;

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
	RDSGUIROSNode rds_gui_ros_node(&n);
	return 0;
}
#ifndef RDS_GUI_ROS_NODE_HPP
#define RDS_GUI_ROS_NODE_HPP

#include <rds/gui.hpp>
#include <rds/geometry.hpp>

#include <rds_network_ros/ToGui.h>

#include <ros/ros.h>

struct RDSGUIROSNode
{
	RDSGUIROSNode(ros::NodeHandle* n);

	void toGuiMessageCallback(const rds_network_ros::ToGui::ConstPtr& to_gui_msg);

	GUI gui_command_space;
	float m_gui_work_space_size;
	GUI gui_work_space;
	ros::Subscriber to_gui_subscriber;
	bool m_draw_orca_circle;

	std::vector<AdditionalPrimitives2D::Arrow> command_space_arrows;
	std::vector<Geometry2D::HalfPlane2> command_space_halfplanes;

	std::vector<Geometry2D::Vec2> solver_space_points;
	std::vector<Geometry2D::HalfPlane2> solver_space_halfplanes;

	//std::vector<Geometry2D::Vec2> work_space_points;
	std::vector<AdditionalPrimitives2D::Circle> work_space_circles;
	std::vector<AdditionalPrimitives2D::Arrow> work_space_arrows;
	std::vector<Geometry2D::Capsule> work_space_capsules;

	GuiColor green, blue, red, magenta, cyan, yellow, orange, blue_fade, white, purple, color_nominal, color_corrected;
};

#endif
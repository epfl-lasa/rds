#include "geometry.hpp"
#include "gui.hpp"

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>

#include <vector>

using Geometry2D::HalfPlane2;
using Geometry2D::Vec2;

std::vector<HalfPlane2> constraints;
std::vector<Vec2> points;

void update_display_distance_minimization(const std_msgs::Float32MultiArray::ConstPtr& rds_gui_msg)
{
	constraints.resize(0);
	points.resize(0);

	Vec2 solution(rds_gui_msg->data[0], rds_gui_msg->data[1]);
	points.push_back(solution);
	points.push_back(Vec2(0.f, 0.f));

	for (int i = 0; i < (rds_gui_msg->data.size()-2)/3; i++)
	{
		constraints.push_back(HalfPlane2(Vec2(rds_gui_msg->data[i*3+2],
			rds_gui_msg->data[i*3+3]), rds_gui_msg->data[i*3+4]));
	}
}

int main(int argc, char** argv)
{
	int dummy_argc = 1;
	char arg1[] = "gui_node";
	char* dummy_argv[] = {arg1};
	ros::init(dummy_argc, dummy_argv, "gui_node");
	ros::NodeHandle n;

	const char topic_name[] = "rds_to_gui";
	ros::Subscriber rds_gui_msg_subscriber = n.subscribe(topic_name, 1000, 
		update_display_distance_minimization);

	GUI gui("RDS Distance Minimization", 1.0);
	gui.halfplanes = &constraints;
	gui.points = &points;
	Window::sdlColor green;
	green.r = green.b = 0;
	green.g = 255;
	gui.points_colors.push_back(green);
	gui.points_colors.push_back(green);

	ros::Rate loop_rate(10);

	while (ros::ok())
	{
		gui.update();
		ros::spinOnce();
		loop_rate.sleep();
	}
}
#include "rds_ros_node.hpp"

#include <rds/geometry.hpp>
#include <rds/capsule.hpp>
#include <rds/rds_4.hpp>

#include <rds_network_ros/ToGui.h>
#include <rds_network_ros/HalfPlane2D.h>
#include <rds_network_ros/Point2D.h>

#define _USE_MATH_DEFINES
#include <cmath>

using Geometry2D::Vec2;
using Geometry2D::Capsule;

bool RDSNode::commandCorrectionService(rds_network_ros::VelocityCommandCorrectionRDS::Request& request,
	rds_network_ros::VelocityCommandCorrectionRDS::Response& response)
{
	MovingCircle moving_object;
	moving_object.velocity = Vec2(0.0, 0.0);
	moving_object.circle.radius = 0.0;
	std::vector<MovingCircle> all_moving_objects;
	for (int i = 0; i < m_aggregator_two_lrf.size(); i++)
	{
		moving_object.circle.center = m_aggregator_two_lrf.getPoint(i);
		all_moving_objects.push_back(moving_object);
	}

	Geometry2D::RDS4 rds_4(1.5, 0.05, 1.2); //tau, delta, v_max

	Capsule robot_shape(0.45, Vec2(0.0, 0.05), Vec2(0.0, -0.5));

	Vec2 p_ref(0.0, 0.25);

	Vec2 v_nominal_p_ref(-p_ref.y*request.nominal_command.angular,
		request.nominal_command.linear + p_ref.x*request.nominal_command.angular);

	Vec2 v_corrected_p_ref;

	rds_4.computeCorrectedVelocity(robot_shape, p_ref,
		v_nominal_p_ref, all_moving_objects, &v_corrected_p_ref);

	response.corrected_command.linear = p_ref.x/p_ref.y*v_corrected_p_ref.x + v_corrected_p_ref.y;
	response.corrected_command.angular = -1.0/p_ref.y*v_corrected_p_ref.x;
	response.got_it = true;

	rds_network_ros::ToGui msg_to_gui;
	msg_to_gui.nominal_command.linear = request.nominal_command.linear;
	msg_to_gui.nominal_command.angular = request.nominal_command.angular;
	msg_to_gui.corrected_command.linear = response.corrected_command.linear;
	msg_to_gui.corrected_command.angular = response.corrected_command.angular;
	msg_to_gui.reference_point.x = p_ref.x;
	msg_to_gui.reference_point.y = p_ref.y;
	msg_to_gui.reference_point_velocity_solution.x = v_corrected_p_ref.x;
	msg_to_gui.reference_point_velocity_solution.y = v_corrected_p_ref.y;
	msg_to_gui.reference_point_nominal_velocity.x = v_nominal_p_ref.x;
	msg_to_gui.reference_point_nominal_velocity.y = v_nominal_p_ref.y;
	rds_network_ros::HalfPlane2D h_msg;
	for (auto& h : rds_4.constraints)
	{
		h_msg.normal.x = h.getNormal().x;
		h_msg.normal.y = h.getNormal().y;
		h_msg.offset = h.getOffset();
		msg_to_gui.reference_point_velocity_constraints.push_back(h_msg);
	}

	rds_network_ros::Point2D p_msg;
	for (auto & mo : all_moving_objects)
	{
		p_msg.x = mo.circle.center.x;
		p_msg.y = mo.circle.center.y;
		msg_to_gui.moving_objects.push_back(p_msg);
	}

	msg_to_gui.robot_shape.radius = robot_shape.radius();
	msg_to_gui.robot_shape.center_a.x = robot_shape.center_a().x;
	msg_to_gui.robot_shape.center_a.y = robot_shape.center_a().y;
	msg_to_gui.robot_shape.center_b.x = robot_shape.center_b().x;
	msg_to_gui.robot_shape.center_b.y = robot_shape.center_b().y;

	publisher_for_gui.publish(msg_to_gui);

	return true;
}

RDSNode::RDSNode(ros::NodeHandle* n, AggregatorTwoLRF& agg)
	: m_aggregator_two_lrf(agg)
	, subscriber_lrf_front(n->subscribe<sensor_msgs::LaserScan>("front_lidar/laserscan"//"sick_laser_front/cropped_scan"//
		, 1, &AggregatorTwoLRF::callbackLRFFront, &m_aggregator_two_lrf))
	, subscriber_lrf_rear(n->subscribe<sensor_msgs::LaserScan>("rear_lidar/laserscan"//"sick_laser_rear/cropped_scan"//
		, 1, &AggregatorTwoLRF::callbackLRFRear, &m_aggregator_two_lrf))
	, publisher_for_gui(n->advertise<rds_network_ros::ToGui>("rds_to_gui", 1)) 
	, command_correction_server(n->advertiseService("rds_velocity_command_correction",
		&RDSNode::commandCorrectionService, this))
{
	ros::spin();
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "rds_ros_node");

	AggregatorTwoLRF aggregator_two_lrf(
		3.f*M_PI/4.f,
		0.05,
		3.f*M_PI/4.f,
		0.05);

	ros::NodeHandle n;
	RDSNode rds_node(&n, aggregator_two_lrf);
	return 0;
}
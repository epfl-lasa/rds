#include "rds_ros_node.hpp"

#include <rds/geometry.hpp>
#include <rds/capsule.hpp>
#include <rds/rds_4.hpp>

#include <rds_network_ros/ToGui.h>
#include <rds_network_ros/HalfPlane2D.h>
#include <rds_network_ros/Point2D.h>
#include <rds_network_ros/Circle.h>

#include <geometry_msgs/TransformStamped.h>

#define _USE_MATH_DEFINES
#include <cmath>

using Geometry2D::Vec2;
using Geometry2D::Capsule;
using AdditionalPrimitives2D::Circle;

int RDSNode::obtainTf(const std::string& frame_id_1, const std::string& frame_id_2, tf2::Transform* tf)
{
	geometry_msgs::TransformStamped transformStamped;
	try
	{
		transformStamped = tf_buffer.lookupTransform(
			frame_id_1, frame_id_2 //"sick_laser_front"//
			, ros::Time(0));
	}
	catch (tf2::TransformException &ex)
	{
		ROS_WARN("%s excpetion, when looking up tf from %s to %s", ex.what(), frame_id_1.c_str(), frame_id_2.c_str());
		return 1;
	}

	tf2::Quaternion rotation(transformStamped.transform.rotation.x,
		transformStamped.transform.rotation.y,
		transformStamped.transform.rotation.z,
		transformStamped.transform.rotation.w);

	tf2::Vector3 translation(transformStamped.transform.translation.x,
		transformStamped.transform.translation.y,
		transformStamped.transform.translation.z);

	*tf = tf2::Transform(rotation, translation);
	return 0;
}

void RDSNode::callbackTracker(const frame_msgs::TrackedPersons::ConstPtr& tracks_msg)
{
	tf2::Transform tf;

	int error_tf_lookup = obtainTf("tf_rds", tracks_msg->header.frame_id, &tf);
	if (error_tf_lookup)
		return;
	tf2::Transform tf_only_rotation(tf.getRotation());

	m_tracked_persons.resize(0);
	tf2::Vector3 position_global, position_local, velocity_global, velocity_local;
	for (const auto& track : tracks_msg->tracks)
	{
		position_global.setX(track.pose.pose.position.x);
		position_global.setY(track.pose.pose.position.y);
		position_global.setZ(track.pose.pose.position.z);

		position_local = tf*position_global;

		velocity_global.setX(track.twist.twist.linear.x);
		velocity_global.setY(track.twist.twist.linear.y);
		velocity_global.setZ(track.twist.twist.linear.z);

		velocity_local = tf_only_rotation*velocity_global;

		m_tracked_persons.push_back(MovingCircle(
			Circle(Vec2(position_local.getX(), position_local.getY()), 0.3), //radius=0.3 (default)
			Vec2(velocity_local.getX(), velocity_local.getY())));
	}
}

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
	for (auto& pedestrian : m_tracked_persons)
		all_moving_objects.push_back(pedestrian);

	float delta = 0.05;
	Geometry2D::RDS4 rds_4(1.5, delta, 1.2); //tau, delta, v_max

	Capsule robot_shape(0.45, Vec2(0.0, 0.05), Vec2(0.0, -0.5));

	Vec2 p_ref(0.0, 0.25);

	Vec2 v_nominal_p_ref(-p_ref.y*request.nominal_command.angular,
		request.nominal_command.linear + p_ref.x*request.nominal_command.angular);

	Vec2 v_corrected_p_ref(0.f, 0.f);

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

	rds_network_ros::Circle c_msg;
	for (auto & mo : all_moving_objects)
	{
		c_msg.center.x = mo.circle.center.x;
		c_msg.center.y = mo.circle.center.y;
		c_msg.radius = mo.circle.radius + delta;
		msg_to_gui.moving_objects.push_back(c_msg);
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
	, subscriber_lrf_front(n->subscribe<sensor_msgs::LaserScan>("front_lidar/scan"//"sick_laser_front/cropped_scan"//
		, 1, &AggregatorTwoLRF::callbackLRFFront, &m_aggregator_two_lrf))
	, subscriber_lrf_rear(n->subscribe<sensor_msgs::LaserScan>("rear_lidar/scan"//"sick_laser_rear/cropped_scan"//
		, 1, &AggregatorTwoLRF::callbackLRFRear, &m_aggregator_two_lrf))
	, subscriber_tracker(n->subscribe<frame_msgs::TrackedPersons>("rwth_tracker/tracked_persons"
		, 1, &RDSNode::callbackTracker, this) )
	, publisher_for_gui(n->advertise<rds_network_ros::ToGui>("rds_to_gui", 1)) 
	, command_correction_server(n->advertiseService("rds_velocity_command_correction",
		&RDSNode::commandCorrectionService, this))
	, tf_listener(tf_buffer)
{
	ros::spin();
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "rds_ros_node");

	AggregatorTwoLRF aggregator_two_lrf(
		3.f*M_PI/4.f,
		0.05,
		100.f,
		3.f*M_PI/4.f,
		0.05,
		100.f);

	ros::NodeHandle n;
	RDSNode rds_node(&n, aggregator_two_lrf);
	return 0;
}

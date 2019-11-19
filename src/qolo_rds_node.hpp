#ifndef QOLO_RDS_NODE_HPP
#define QOLO_RDS_NODE_HPP

#include "qolo_collision_point_generator.hpp"

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>

struct QoloRDSNode
{
	QoloRDSNode(ros::NodeHandle* n);

	void nominalCommandCallbackToPublishCorrectedCommand(const std_msgs::Float32MultiArray::ConstPtr& nominal_command_msg);

	QoloCollisionPointGenerator qolo_collision_point_generator;

	ros::Publisher corrected_command_publisher;
	ros::Publisher publisher_for_gui;
	ros::Subscriber nominal_command_subscriber;
	ros::Subscriber lrf_subscriber;
};

#endif
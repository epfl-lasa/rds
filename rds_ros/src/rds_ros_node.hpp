#ifndef RDS_ROS_NODE_HPP
#define RDS_ROS_NODE_HPP

#include "aggregate_two_lrf.hpp"
#include <rds/geometry.hpp>

#include <rds_network_ros/VelocityCommandCorrectionRDS.h>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

struct RDSNode
{
	RDSNode(ros::NodeHandle* n, AggregatorTwoLRF& agg);

	bool commandCorrectionService(rds_network_ros::VelocityCommandCorrectionRDS::Request& request,
		rds_network_ros::VelocityCommandCorrectionRDS::Response& response);

	AggregatorTwoLRF& m_aggregator_two_lrf;
	ros::Subscriber subscriber_lrf_front;
	ros::Subscriber subscriber_lrf_rear;
	ros::Publisher publisher_for_gui;
	ros::ServiceServer command_correction_server;
};

#endif
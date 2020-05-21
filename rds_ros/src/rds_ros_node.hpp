#ifndef RDS_ROS_NODE_HPP
#define RDS_ROS_NODE_HPP

#include "aggregate_two_lrf.hpp"
#include <rds/geometry.hpp>
#include <rds/rds_4.hpp>

#include <rds_network_ros/VelocityCommandCorrectionRDS.h>

#include <frame_msgs/TrackedPersons.h>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.h>

#include <tf2_ros/message_filter.h>
#include <message_filters/subscriber.h>

#include <vector>
#include <string>

struct RDSNode
{
	RDSNode(ros::NodeHandle* n, AggregatorTwoLRF& agg);

	bool commandCorrectionService(rds_network_ros::VelocityCommandCorrectionRDS::Request& request,
		rds_network_ros::VelocityCommandCorrectionRDS::Response& response);

	void callbackTracker(const frame_msgs::TrackedPersons::ConstPtr& tracks_msg);

	int obtainTrackerMsgRelativeTf(const frame_msgs::TrackedPersons::ConstPtr& tracks_msg,
		tf2::Transform* tf);

	AggregatorTwoLRF& m_aggregator_two_lrf;
	ros::Subscriber subscriber_lrf_front;
	ros::Subscriber subscriber_lrf_rear;

	const std::string target_frame;
	tf2_ros::Buffer tf_buffer;
	tf2_ros::TransformListener tf_listener;
	message_filters::Subscriber<frame_msgs::TrackedPersons> subscriber_tracker;
	tf2_ros::MessageFilter<frame_msgs::TrackedPersons> tracker_msg_filter;

	ros::Publisher publisher_for_gui;
	ros::ServiceServer command_correction_server;
	std::vector<MovingCircle> m_tracked_persons;
};

#endif
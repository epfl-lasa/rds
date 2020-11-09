#ifndef RDS_ROS_NODE_HPP
#define RDS_ROS_NODE_HPP

#include "aggregate_two_lrf.hpp"
#include <rds/geometry.hpp>
#include <rds/rds_5.hpp>

#include <rds_network_ros/VelocityCommandCorrectionRDS.h>

#ifdef RDS_ROS_USE_TRACKER
	#include <frame_msgs/TrackedPersons.h>
#endif

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.h>

#include <vector>
#include <string>

#include <chrono>

#ifdef RDS_ROS_USE_TRACKER
	struct MovingObject3
	{
		tf2::Vector3 position, velocity;
	};

	struct PersonTracks
	{
		PersonTracks() { }
		PersonTracks(const frame_msgs::TrackedPersons::ConstPtr& tracker_message);
		void updatePositions(const std::chrono::time_point<std::chrono::high_resolution_clock>& time_now);
		const std::vector<MovingObject3>& getPersonsGlobal() const { return persons_global; }
		const std::string& getFrameId() const { return frame_id; }

		std::vector<MovingCircle> persons_local;
	private:
		std::vector<MovingObject3> persons_global;
		std::chrono::time_point<std::chrono::high_resolution_clock> time;
		std::string frame_id;
		float delay;
	};
#endif

struct RDSNode
{
	RDSNode(ros::NodeHandle* n, AggregatorTwoLRF& agg);

	bool commandCorrectionService(rds_network_ros::VelocityCommandCorrectionRDS::Request& request,
		rds_network_ros::VelocityCommandCorrectionRDS::Response& response);

#ifdef RDS_ROS_USE_TRACKER
	void callbackTracker(const frame_msgs::TrackedPersons::ConstPtr& tracks_msg);

	int obtainTf(const std::string& frame_id_1, const std::string& frame_id_2, tf2::Transform* tf);

	int makeLocalPersons(const std::vector<MovingObject3>& persons_global,
		const std::string& tracks_frame_id, std::vector<MovingCircle>* persons_local);
#endif

	AggregatorTwoLRF& m_aggregator_two_lrf;
	ros::Subscriber subscriber_lrf_front;
	ros::Subscriber subscriber_lrf_rear;

#ifdef RDS_ROS_USE_TRACKER
	ros::Subscriber subscriber_tracker;
#endif

	ros::Publisher publisher_for_gui;
	ros::ServiceServer command_correction_server;

#ifdef RDS_ROS_USE_TRACKER
	std::vector<MovingCircle> m_tracked_persons;
	PersonTracks m_person_tracks;
#endif
	
	tf2_ros::Buffer tf_buffer;
	tf2_ros::TransformListener tf_listener;
	float command_correct_previous_linear, command_correct_previous_angular;
	unsigned int call_counter;
};

#endif
#ifndef LIDAR_CHANNELS_TO_LRF_NODE_HPP
#define LIDAR_CHANNELS_TO_LRF_NODE_HPP

#include <ros/ros.h>
#include <velodyne_msgs/VelodyneScan.h>
#include <velodyne_pointcloud/rawdata.h>

#include <vector>

struct LidarChannelsToLRFNode
{
	LidarChannelsToLRFNode(ros::NodeHandle* n);

	void velodyneMessageCallback(const velodyne_msgs::VelodyneScan::ConstPtr& v_scan);

	ros::Publisher lrf_publisher;
	ros::Subscriber velodyne_subscriber;

	const unsigned int channel_index_1, channel_index_2;
	const int lrf_number_of_bins;
	const float lrf_angle_step;

	struct RingPoint3D
	{
		RingPoint3D(float x, float y, float z, unsigned int ring_index) : x(x), y(y), z(z), ring_index(ring_index) { }
		float x,y,z;
		unsigned int ring_index;
	};

	struct PointCloud3D : public velodyne_rawdata::DataContainerBase
	{
		virtual void addPoint(const float& x, const float& y, const float& z,
			const uint16_t& ring, const uint16_t& azimuth, const float& distance,
			const float& intensity)
		{
			points.push_back(RingPoint3D(x, y, z, ring));
		}

		std::vector<RingPoint3D> points;
	};
};

#endif
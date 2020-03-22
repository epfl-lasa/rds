#include "realsense_to_lrf_node.hpp"

#include <sensor_msgs/LaserScan.h>

#include <ros/package.h> // to find the path to the calibration file in the velodyne package

#include <string>

#define _USE_MATH_DEFINES
#include <cmath>

float positive_angle(float angle)
{
	while (angle < 0.f)
		angle += 2.f*M_PI;
	return angle;
}

void RealSenseoLRFNode::velodyneMessageCallback(const velodyne_msgs::VelodyneScan::ConstPtr& v_scan)
{
	std::string velodyne_pointcloud_package_path = ros::package::getPath("velodyne_pointcloud");
	std::string calibration_file_path = velodyne_pointcloud_package_path + "/params/VLP16db.yaml";
	velodyne_rawdata::RawData raw_data;
	raw_data.setupOffline(calibration_file_path, 10000.f, 0.f);
	raw_data.setParameters(0.f, 10000.f, 0.f, 2.f*M_PI);

	LidarToLRFNode::PointCloud3D point_cloud;
	for (auto& packet : v_scan->packets)
		raw_data.unpack(packet, point_cloud);

	// rotate around x-axis to compensate forward tilting
	float ryy = std::cos(-lidar_tilt_angle);
	float ryz = std::sin(-lidar_tilt_angle);
	float rzy = -ryz;
	float rzz = ryy;
	float y_new, z_new;
	for (auto& p : point_cloud.points)
	{
		y_new = ryy*p.y + rzy*p.z;
		z_new = ryz*p.y + rzz*p.z;
		p.y = y_new;
		p.z = z_new;
	}

	// create a 2D laserscan message (skip ground scans)
	sensor_msgs::LaserScan lrf_scan;
	lrf_scan.header.frame_id = "velodyne";
	lrf_scan.header.stamp = v_scan->header.stamp;
	lrf_scan.angle_increment = lrf_angle_step;
	lrf_scan.angle_min = 0.f;
	lrf_scan.angle_max = 2.f*M_PI;
	lrf_scan.range_min = 0.f;
	lrf_scan.range_max = 10000.f;
	lrf_scan.ranges = std::vector<float>(lrf_number_of_bins, 100000.f);
	for (auto& p : point_cloud.points)
	{
		if (p.z > -lidar_mount_height)
		//if ((p.z > -0.04f) && (p.z < 0.04f))
		{
			float phi = std::atan2(p.y, p.x);
			float r = std::sqrt(p.x*p.x + p.y*p.y);
			int lrf_bin_index = positive_angle(phi)/lrf_angle_step;
			if (lrf_scan.ranges[lrf_bin_index] > r)
				lrf_scan.ranges[lrf_bin_index] = r;
		}
	}
	lrf_publisher.publish(lrf_scan);
}

LidarToLRFNode::LidarToLRFNode(ros::NodeHandle* n)
	: lrf_publisher(n->advertise<sensor_msgs::LaserScan>("laserscan", 1))
	, velodyne_subscriber(n->subscribe<velodyne_msgs::VelodyneScan>("velodyne_packets", 1,
		&LidarToLRFNode::velodyneMessageCallback, this))
	, lidar_mount_height(0.35f)//0.48 would be exact
	, lidar_tilt_angle(2.f/360.f*2.f*M_PI)
	, lrf_number_of_bins(720)
	, lrf_angle_step(2.f*M_PI/lrf_number_of_bins)
{
	ros::spin();
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "ros_realsense_to_lrf_node");
	ros::NodeHandle n;
	RealsSenseToLRFNode realsense_to_lrf_node(&n);
	return 0;
}

// to visualize the result in RViz:
// rosrun tf static_transform_publisher 0 0 0 0 0 0 1 map velodyne 10
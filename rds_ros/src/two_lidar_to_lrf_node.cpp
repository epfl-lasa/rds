#include "two_lidar_to_lrf_node.hpp"

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

TwoLidarToLRFNode::TwoLidarToLRFNode(ros::NodeHandle* n)
	: lrf_publisher(n->advertise<sensor_msgs::LaserScan>("laserscan", 1))
	, front_velodyne_subscriber(n->subscribe<velodyne_msgs::VelodyneScan>("front_velodyne_packets", 1,
		&TwoLidarToLRFNode::frontVelodyneMessageCallback, this))
	, rear_velodyne_subscriber(n->subscribe<velodyne_msgs::VelodyneScan>("rear_velodyne_packets", 1,
		&TwoLidarToLRFNode::rearVelodyneMessageCallback, this))
	, front_lidar_y_coordinate(0.056)
	, front_lidar_mount_height(0.35f)//0.48 would be exact
	, front_lidar_tilt_angle(2.f/360.f*2.f*M_PI)
	, rear_lidar_y_coordinate(-0.517)
	, rear_lidar_mount_height(0.f) // UNKNOWN
	, rear_lidar_tilt_angle(front_lidar_tilt_angle) // UNKNOWN
	, lrf_y_coordinate(-0.2305) // just in the middle between the two lidars
	, lrf_number_of_bins(720)
	, lrf_angle_step(2.f*M_PI/lrf_number_of_bins)
{
	received_one_velodyne_scan = false;
	lrf_scan.header.frame_id = "velodyne";
	lrf_scan.angle_increment = lrf_angle_step;
	lrf_scan.angle_min = 0.f;
	lrf_scan.angle_max = 2.f*M_PI;
	lrf_scan.range_min = 0.f;
	lrf_scan.range_max = 10000.f;
	ros::spin();
}

void TwoLidarToLRFNode::frontVelodyneMessageCallback(const velodyne_msgs::VelodyneScan::ConstPtr& v_scan)
{
	std::string velodyne_pointcloud_package_path = ros::package::getPath("velodyne_pointcloud");
	std::string calibration_file_path = velodyne_pointcloud_package_path + "/params/VLP16db.yaml";
	velodyne_rawdata::RawData raw_data;
	raw_data.setupOffline(calibration_file_path, 10000.f, 0.f);
	raw_data.setParameters(0.f, 10000.f, 0.f, 2.f*M_PI);

	TwoLidarToLRFNode::PointCloud3D point_cloud;
	for (auto& packet : v_scan->packets)
		raw_data.unpack(packet, point_cloud);

	// rotate around x-axis to compensate forward tilting
	float ryy = std::cos(-front_lidar_tilt_angle);
	float ryz = std::sin(-front_lidar_tilt_angle);
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

	// fill the 2D laserscan message (skip ground scans)
	if (!received_one_velodyne_scan)
	{
		lrf_scan.header.stamp = v_scan->header.stamp;
		lrf_scan.ranges = std::vector<float>(lrf_number_of_bins, 100000.f);
	}
	for (auto& p : point_cloud.points)
	{
		if (p.z > -front_lidar_mount_height)
		//if ((p.z > -0.04f) && (p.z < 0.04f))
		{
			float p_y_lrf = p.y + front_lidar_y_coordinate - lrf_y_coordinate;
			float phi = std::atan2(p_y_lrf, p.x);
			float r = std::sqrt(p.x*p.x + p_y_lrf*p_y_lrf);
			int lrf_bin_index = positive_angle(phi)/lrf_angle_step;
			if (lrf_scan.ranges[lrf_bin_index] > r)
				lrf_scan.ranges[lrf_bin_index] = r;
		}
	}
	if (!received_one_velodyne_scan)
		received_one_velodyne_scan = true;
	else
		lrf_publisher.publish(lrf_scan);
}

void TwoLidarToLRFNode::rearVelodyneMessageCallback(const velodyne_msgs::VelodyneScan::ConstPtr& v_scan)
{
	std::string velodyne_pointcloud_package_path = ros::package::getPath("velodyne_pointcloud");
	std::string calibration_file_path = velodyne_pointcloud_package_path + "/params/VLP16db.yaml";
	velodyne_rawdata::RawData raw_data;
	raw_data.setupOffline(calibration_file_path, 10000.f, 0.f);
	raw_data.setParameters(0.f, 10000.f, 0.f, 2.f*M_PI);

	TwoLidarToLRFNode::PointCloud3D point_cloud;
	for (auto& packet : v_scan->packets)
		raw_data.unpack(packet, point_cloud);

	// rotate everything around z-axis by 180 degree
	// rotate around x-axis to compensate forward tilting
	{
		float rxx = std::cos(M_PI);
		float rxy = std::sin(M_PI);
		float ryx = -rxy;
		float ryy = rxx;
		float x_new, y_new;
		for (auto& p : point_cloud.points)
		{
			x_new = rxx*p.x + ryx*p.y;
			y_new = rxy*p.x + ryy*p.y;
			p.x = x_new;
			p.y = y_new;
		}
	}

	// rotate around x-axis to compensate forward tilting
	{
		float ryy = std::cos(-rear_lidar_tilt_angle);
		float ryz = std::sin(-rear_lidar_tilt_angle);
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
	}

	// fill the 2D laserscan message (skip ground scans)
	if (!received_one_velodyne_scan)
	{
		lrf_scan.header.stamp = v_scan->header.stamp;
		lrf_scan.ranges = std::vector<float>(lrf_number_of_bins, 100000.f);
	}
	for (auto& p : point_cloud.points)
	{
		if (p.z > -rear_lidar_mount_height)
		//if ((p.z > -0.04f) && (p.z < 0.04f))
		{
			float p_y_lrf = p.y + rear_lidar_y_coordinate - lrf_y_coordinate;
			float phi = std::atan2(p_y_lrf, p.x);
			float r = std::sqrt(p.x*p.x + p_y_lrf*p_y_lrf);
			int lrf_bin_index = positive_angle(phi)/lrf_angle_step;
			if (lrf_scan.ranges[lrf_bin_index] > r)
				lrf_scan.ranges[lrf_bin_index] = r;
		}
	}
	if (!received_one_velodyne_scan)
		received_one_velodyne_scan = true;
	else
		lrf_publisher.publish(lrf_scan);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "rds_ros_two_lidar_to_lrf_node");
	ros::NodeHandle n;
	TwoLidarToLRFNode two_lidar_to_lrf_node(&n);
	return 0;
}
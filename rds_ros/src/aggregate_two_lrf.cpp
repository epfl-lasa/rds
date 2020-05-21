#include "aggregate_two_lrf.hpp"

#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Transform.h>

#define _USE_MATH_DEFINES
#include <cmath>

using Geometry2D::Vec2;

unsigned int AggregatorTwoLRF::size()
{
	return points_front.size() + points_rear.size();
}

const Vec2& AggregatorTwoLRF::getPoint(unsigned int index)
{
	if (index < points_front.size())
		return points_front[index];
	else if (index < this->size())
		return points_rear[index - points_front.size()];
	else
		throw IndexOutOfRangeException();
}

float angle_difference(float alpha, float beta)
{
	while (alpha - beta > M_PI)
		beta += 2.f*M_PI;
	while (alpha - beta < -M_PI)
		beta -= 2.f*M_PI;
	return alpha - beta;
}

void AggregatorTwoLRF::getPointsFromLRF(const sensor_msgs::LaserScan::ConstPtr& lrf_msg,
	float angle_cutoff, float range_cutoff_lower, float range_cutoff_upper,
	std::vector<Geometry2D::Vec2>* result_points)
{
	geometry_msgs::TransformStamped transformStamped;
	try
	{
		transformStamped = tf_buffer.lookupTransform(
			 "tf_rds", lrf_msg->header.frame_id //"sick_laser_front"//
			, ros::Time(0));
	}
	catch (tf2::TransformException &ex)
	{
		ROS_WARN("%s excpetion, when looking up tf from %s to tf_rds", ex.what(), lrf_msg->header.frame_id);
		return;
	}

	tf2::Quaternion rotation(transformStamped.transform.rotation.x,
		transformStamped.transform.rotation.y,
		transformStamped.transform.rotation.z,
		transformStamped.transform.rotation.w);

	tf2::Vector3 translation(transformStamped.transform.translation.x,
		transformStamped.transform.translation.y,
		transformStamped.transform.translation.z);

	tf2::Transform transform(rotation, translation);

	result_points->resize(0);
	for (std::vector<float>::size_type i = 0; i != lrf_msg->ranges.size(); i++)
	{
		float phi = lrf_msg->angle_min + i*lrf_msg->angle_increment;
		if (angle_cutoff < std::abs(angle_difference(0.0, phi)))
			continue;
		if (range_cutoff_lower > lrf_msg->ranges[i])
			continue;
		if (range_cutoff_upper < lrf_msg->ranges[i])
			continue;
		if (lrf_msg->ranges[i] != lrf_msg->ranges[i]) //true for nan
			continue;

		tf2::Vector3 position_lrf_frame(lrf_msg->ranges[i]*std::cos(phi), lrf_msg->ranges[i]*std::sin(phi), 0.0);
		tf2::Vector3 position_axle_frame = transform.operator*(position_lrf_frame);

		result_points->push_back(Vec2(position_axle_frame.getX(), position_axle_frame.getY()));
	}
}

AggregatorTwoLRF::AggregatorTwoLRF(float angle_cutoff_lrf_front, float range_cutoff_lower_lrf_front , float range_cutoff_upper_lrf_front,
	float angle_cutoff_lrf_rear, float range_cutoff_lower_lrf_rear, float range_cutoff_upper_lrf_rear)
	: angle_cutoff_lrf_front(angle_cutoff_lrf_front)
	, range_cutoff_lower_lrf_front(range_cutoff_lower_lrf_front)
	, range_cutoff_upper_lrf_front(range_cutoff_upper_lrf_front)
	, angle_cutoff_lrf_rear(angle_cutoff_lrf_rear)
	, range_cutoff_lower_lrf_rear(range_cutoff_lower_lrf_rear)
	, range_cutoff_upper_lrf_rear(range_cutoff_upper_lrf_rear)
	, tf_listener(tf_buffer)
{ }

void AggregatorTwoLRF::callbackLRFFront(const sensor_msgs::LaserScan::ConstPtr& lrf_msg)
{
	getPointsFromLRF(lrf_msg, angle_cutoff_lrf_front, range_cutoff_lower_lrf_front,
		range_cutoff_upper_lrf_front, &points_front);
}

void AggregatorTwoLRF::callbackLRFRear(const sensor_msgs::LaserScan::ConstPtr& lrf_msg)
{
	getPointsFromLRF(lrf_msg, angle_cutoff_lrf_rear, range_cutoff_lower_lrf_rear,
		range_cutoff_upper_lrf_rear, &points_rear);
}

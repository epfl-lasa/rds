#include "aggregate_two_lrf.hpp"

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
	const Geometry2D::Vec2& position_lrf, float orientation_lrf,
	float angle_center, float angle_cutoff, float range_cutoff_lower,
	std::vector<Geometry2D::Vec2>* result_points)
{
	result_points->resize(0);
	for (std::vector<float>::size_type i = 0; i != lrf_msg->ranges.size(); i++)
	{
		float phi = orientation_lrf + lrf_msg->angle_min + i*lrf_msg->angle_increment;
		if (angle_cutoff < std::abs(angle_difference(angle_center, phi)))
			continue;
		if (range_cutoff_lower > lrf_msg->ranges[i])
			continue;
		Vec2 point = position_lrf + lrf_msg->ranges[i]*Vec2(std::cos(phi), std::sin(phi));
		result_points->push_back(point);
	}
}

AggregatorTwoLRF::AggregatorTwoLRF(const Geometry2D::Vec2& position_lrf_front, float orientation_lrf_front,
	float angle_center_lrf_front, float angle_cutoff_lrf_front, float range_cutoff_lower_lrf_front,
	const Geometry2D::Vec2& position_lrf_rear, float orientation_lrf_rear,
	float angle_center_lrf_rear, float angle_cutoff_lrf_rear, float range_cutoff_lower_lrf_rear)
	: position_lrf_front(position_lrf_front)
	, orientation_lrf_front(orientation_lrf_front)
	, angle_center_lrf_front(angle_center_lrf_front)
	, angle_cutoff_lrf_front(angle_cutoff_lrf_front)
	, range_cutoff_lower_lrf_front(range_cutoff_lower_lrf_front)
	, position_lrf_rear(position_lrf_rear)
	, orientation_lrf_rear(orientation_lrf_rear)
	, angle_center_lrf_rear(angle_center_lrf_rear)
	, angle_cutoff_lrf_rear(angle_cutoff_lrf_rear)
	, range_cutoff_lower_lrf_rear(range_cutoff_lower_lrf_rear)
{ }

void AggregatorTwoLRF::callbackLRFFront(const sensor_msgs::LaserScan::ConstPtr& lrf_msg)
{
	getPointsFromLRF(lrf_msg, position_lrf_front, orientation_lrf_front,
		angle_center_lrf_front, angle_cutoff_lrf_front, range_cutoff_lower_lrf_front,
		&points_front);
}

void AggregatorTwoLRF::callbackLRFRear(const sensor_msgs::LaserScan::ConstPtr& lrf_msg)
{
	getPointsFromLRF(lrf_msg, position_lrf_rear, orientation_lrf_rear,
		angle_center_lrf_rear, angle_cutoff_lrf_rear, range_cutoff_lower_lrf_rear,
		&points_rear);
}
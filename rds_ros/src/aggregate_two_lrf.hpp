#ifndef AGGREGATE_TWO_LRF_HPP
#define AGGREGATE_TWO_LRF_HPP

#include "rds/geometry.hpp"
#include <sensor_msgs/LaserScan.h>
#include <vector>

struct AggregatorTwoLRF
{
	AggregatorTwoLRF(const Geometry2D::Vec2& position_lrf_front,
		float orientation_lrf_front,
		float angle_center_lrf_front,
		float angle_cutoff_lrf_front,
		float range_cutoff_lower_lrf_front,
		const Geometry2D::Vec2& position_lrf_rear,
		float orientation_lrf_rear,
		float angle_center_lrf_rear,
		float angle_cutoff_lrf_rear,
		float range_cutoff_lower_lrf_rear);

	unsigned int size();
	const Geometry2D::Vec2& getPoint(unsigned int index);

	void callbackLRFFront(const sensor_msgs::LaserScan::ConstPtr& lrf_msg);
	void callbackLRFRear(const sensor_msgs::LaserScan::ConstPtr& lrf_msg);

	std::vector<Geometry2D::Vec2> points_front;
	std::vector<Geometry2D::Vec2> points_rear;

	// parameters lrf front
	const Geometry2D::Vec2 position_lrf_front;
	const float orientation_lrf_front;
	const float angle_center_lrf_front;
	const float angle_cutoff_lrf_front;
	const float range_cutoff_lower_lrf_front;
	// parameters lrf rear
	const Geometry2D::Vec2 position_lrf_rear;
	const float orientation_lrf_rear;
	const float angle_center_lrf_rear;
	const float angle_cutoff_lrf_rear;
	const float range_cutoff_lower_lrf_rear;

	struct IndexOutOfRangeException { };

private:
	void getPointsFromLRF(const sensor_msgs::LaserScan::ConstPtr& lrf_msg,
		const Geometry2D::Vec2& position_lrf, float orientation_lrf,
		float angle_center, float angle_cutoff, float range_cutoff_lower,
		std::vector<Geometry2D::Vec2>* result_points);
};

/*struct AggregatorTwoLRFDepthCamera : public AggregatorTwoLRF
{
	std::vector<Geometry2D::Vec2> points_depth_camera;
};*/

#endif
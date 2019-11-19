#include "rds_wrap.hpp"
#include "rds_core.hpp"

const CollisionPointGenerator& CollisionPointGenerator::generateCollisionPoints()
{
	updateObstacleCirclesAndVelocities();
	collision_points.resize(obstacle_circles.size()*robot_shape_circles.size());
	for (std::vector<AdditionalPrimitives2D::Circle>::size_type j = 0; j != robot_shape_circles.size(); j++)
	{
		for (std::vector<AdditionalPrimitives2D::Circle>::size_type i = 0; i != obstacle_circles.size(); i++)
		{
			collision_points[j*obstacle_circles.size() + i] = RDS::CollisionPoint(robot_shape_circles[j],
				obstacle_circles[i], obstacle_velocities[i]);
		}
	}
	return *this;
}

void CollisionPointGenerator::LRFCallbackToUpdateObstacleCirclesAndVelocities(const sensor_msgs::LaserScan::ConstPtr& laserscan_msg)
{
	obstacle_circles.resize(laserscan_msg->ranges.size());
	obstacle_velocities.resize(laserscan_msg->ranges.size());
	for (std::vector<float>::size_type i = 0; i != range_scan.size(); i++)
	{
		obstacle_velocities[i] = Geometry2D::Vec2(0.f, 0.f);
		float phi = lrf_orientation + laserscan_msg->angle_min + i*laserscan_msg->angle_increment;
		Geometry2D::Vec2 center(lrf_location + laserscan_msg->ranges[i]*Geometry2D::Vec2(std::cos(phi),
			std::sin(phi)));
		obstacle_circles[i] = AdditionalPrimitives2D::Circle(center, 0.f);
	}
}


RDSWrap::RDSWrap(const RDS::VelocityCommand& nominal_command,
	const RDS::VelocityCommandBoxLimits& box_limits,
	const RDS::VelocityCommandHexagonLimits& hexagon_limits,
	const std::vector<RDS::CollisionPoint>& collision_points,
	float y_coordinate_of_reference_point_for_command_limits,
	float weight_scaling_of_reference_point_for_command_limits,
	float tau,
	float delta,
	float clearance_from_axle_of_final_reference_point)
	: pvcg(box_limits,
		hexagon_limits,
		collision_points,
		y_coordinate_of_reference_point_for_command_limits,
		tau,
		delta)
	, rpg(nominal_command,
		weight_scaling_of_reference_point_for_command_limits,
		clearance_from_axle_of_final_reference_point,
		pvcg)
	, rpvcc(rpg,
		pvcg)
	, rpvo(nominal_command,
		hexagon_limits,
		rpvcc,
		rpg)
{ }

RDSWrap::RDSWrap(const RDS::VelocityCommand& nominal_command,
	const RDS::VelocityCommandBoxLimits& box_limits,
	const RDS::VelocityCommandHexagonLimits& hexagon_limits,
	CollisionPointGenerator* collision_point_generator,
	float y_coordinate_of_reference_point_for_command_limits,
	float weight_scaling_of_reference_point_for_command_limits,
	float tau,
	float delta,
	float clearance_from_axle_of_final_reference_point)
	: pvcg(box_limits,
		hexagon_limits,
		collision_point_generator->generateCollisionPoints().collision_points,
		y_coordinate_of_reference_point_for_command_limits,
		tau,
		delta)
	, rpg(nominal_command,
		weight_scaling_of_reference_point_for_command_limits,
		clearance_from_axle_of_final_reference_point,
		pvcg)
	, rpvcc(rpg,
		pvcg)
	, rpvo(nominal_command,
		hexagon_limits,
		rpvcc,
		rpg)
{ }
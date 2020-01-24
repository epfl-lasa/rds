#include "rds_holonomic_ros_node.hpp"

#include <rds/differential_drive_kinematics.hpp>
#include <rds/collision_point.hpp>

#include <rds_network_ros/ToGui.h>
#include <rds_network_ros/HalfPlane2D.h>
#include <rds_network_ros/Point2D.h>
#include <rds_network_ros/Circle.h>

#define _USE_MATH_DEFINES
#include <cmath>

MyCollisionPointGenerator::MyCollisionPointGenerator()
	: front_lrf_location(Geometry2D::Vec2(0.f, 0.056f))
	, front_lrf_orientation(M_PI/2.f)
	, front_angle_cutoff_from_forward_direction(3.f*M_PI/4.f)
	, front_range_cutoff_lower(0.05f)
{
	defineMyShape();
}

void MyCollisionPointGenerator::defineMyShape()
{
	// create a shape (in robot-fixed coordinates, x is lateral, y is forward)
	Geometry2D::Vec2 center(0.f, 0.f);
	float radius = 0.4f;
	robot_shape_circles.push_back(AdditionalPrimitives2D::Circle(center, radius));
	return;
}

float angleToPlus270Minus90(float angle)
{
	while (angle < -M_PI/2.f)
		angle += 2.f*M_PI;
	while (angle > 3.f*M_PI/2.f)
		angle -= 2.f*M_PI;
	return angle;
}

void MyCollisionPointGenerator::obstacleMessageCallback(const sensor_msgs::LaserScan::ConstPtr& lrf_msg)
{
	obstacle_circles.resize(0);//lrf_msg->ranges.size());
	obstacle_velocities.resize(0);//lrf_msg->ranges.size());
	for (std::vector<float>::size_type i = 0; i != lrf_msg->ranges.size(); i++)
	{
		//obstacle_velocities[i] = Geometry2D::Vec2(0.f, 0.f);
		float phi = front_lrf_orientation + lrf_msg->angle_min + i*lrf_msg->angle_increment;
		Geometry2D::Vec2 center(front_lrf_location + lrf_msg->ranges[i]*Geometry2D::Vec2(std::cos(phi),
			std::sin(phi)));
		//if (lrf_msg->ranges[i] > 1.f)
		//obstacle_circles[i] = AdditionalPrimitives2D::Circle(center, 0.f);
		if ((std::abs(angleToPlus270Minus90(phi) - M_PI/2.f) < front_angle_cutoff_from_forward_direction) &&
			(lrf_msg->ranges[i] > front_range_cutoff_lower))
		{
			obstacle_circles.push_back(AdditionalPrimitives2D::Circle(center, 0.f));
			obstacle_velocities.push_back(Geometry2D::Vec2(0.f, 0.f));
		}
	}
}

bool RDSHolonomicNode::commandCorrectionService(rds_network_ros::VelocityCommandCorrectionRDS::Request& request,
	rds_network_ros::VelocityCommandCorrectionRDS::Response& response)
{
	RDS::VelocityCommand nominal_command(request.nominal_command.linear, request.nominal_command.angular);

	RDS::VelocityCommandHexagonLimits hexagon_limits;
	hexagon_limits.min_linear = request.velocity_limits.min_linear;
	hexagon_limits.max_linear = request.velocity_limits.max_linear;
	hexagon_limits.absolute_angular_at_min_linear = request.velocity_limits.abs_angular_at_min_linear;
	hexagon_limits.absolute_angular_at_max_linear = request.velocity_limits.abs_angular_at_max_linear;
	hexagon_limits.absolute_angular_at_zero_linear = request.velocity_limits.abs_angular_at_zero_linear;

	RDS::VelocityCommandBoxLimits box_limits;
	box_limits.min_linear = request.last_actual_command.linear - request.command_cycle_time*request.abs_linear_acceleration_limit;
	box_limits.min_angular = request.last_actual_command.angular - request.command_cycle_time*request.abs_angular_acceleration_limit;
	box_limits.max_linear = request.last_actual_command.linear + request.command_cycle_time*request.abs_linear_acceleration_limit;
	box_limits.max_angular = request.last_actual_command.angular + request.command_cycle_time*request.abs_angular_acceleration_limit;

	/*
	float y_coordinate_of_reference_point_for_command_limits = 0.5f;
	float weight_scaling_of_reference_point_for_command_limits = 1.f;
	float tau = 2.f;
	float delta = 0.05f;
	float clearance_from_axle_of_final_reference_point = 0.15f;
	*/
	float y_coordinate_of_reference_point_for_command_limits = request.y_coordinate_of_reference_point_for_command_limits;
	float weight_scaling_of_reference_point_for_command_limits = request.weight_scaling_of_reference_point_for_command_limits;
	float tau = request.tau;
	float delta = request.delta;
	float clearance_from_axle_of_final_reference_point = request.clearance_from_axle_of_final_reference_point;

	float y_coordinate_of_reference_biasing_point = request.y_coordinate_of_reference_biasing_point;
	float weight_of_reference_biasing_point = request.weight_of_reference_biasing_point;

	RDSWrap rds_wrap(nominal_command,
		box_limits,
		hexagon_limits,
		my_cpg.generateCollisionPoints().collision_points,
		y_coordinate_of_reference_point_for_command_limits,
		weight_scaling_of_reference_point_for_command_limits,
		tau,
		delta,
		clearance_from_axle_of_final_reference_point,
		y_coordinate_of_reference_biasing_point,
		weight_of_reference_biasing_point,
		false,
		true); // activate holonomic case

	response.corrected_command.linear = rds_wrap.getCommandSolution().linear;
	response.corrected_command.angular = rds_wrap.getCommandSolution().angular;
	response.feasible = rds_wrap.isFeasible();

	rds_network_ros::ToGui msg_to_gui;
	msg_to_gui.nominal_command.linear = nominal_command.linear;
	msg_to_gui.nominal_command.angular = nominal_command.angular;
	msg_to_gui.corrected_command.linear = rds_wrap.getCommandSolution().linear;
	msg_to_gui.corrected_command.angular = rds_wrap.getCommandSolution().angular;
	msg_to_gui.reference_point.x = rds_wrap.getReferencePoint().x;
	msg_to_gui.reference_point.y = rds_wrap.getReferencePoint().y;
	msg_to_gui.reference_point_velocity_solution.x = rds_wrap.getReferencePointVelocitySolution().x;
	msg_to_gui.reference_point_velocity_solution.y = rds_wrap.getReferencePointVelocitySolution().y;
	msg_to_gui.reference_point_nominal_velocity.x = nominal_command.pointVelocity(rds_wrap.getReferencePoint()).x;
	msg_to_gui.reference_point_nominal_velocity.y = nominal_command.pointVelocity(rds_wrap.getReferencePoint()).y;
	rds_network_ros::HalfPlane2D h_msg;
	for (auto& h : rds_wrap.getReferencePointVelocityConstraints())
	{
		h_msg.normal.x = h.getNormal().x;
		h_msg.normal.y = h.getNormal().y;
		h_msg.offset = h.getOffset();
		msg_to_gui.reference_point_velocity_constraints.push_back(h_msg);
	}
	msg_to_gui.solver_solution.x = rds_wrap.getScaledShiftedSolution().x;
	msg_to_gui.solver_solution.y = rds_wrap.getScaledShiftedSolution().y;
	for (auto& h : rds_wrap.getScaledShiftedConstraints())
	{
		h_msg.normal.x = h.getNormal().x;
		h_msg.normal.y = h.getNormal().y;
		h_msg.offset = h.getOffset();
		msg_to_gui.solver_constraints.push_back(h_msg);
	}
	msg_to_gui.reference_point_for_command_limits.x = 0.f;
	msg_to_gui.reference_point_for_command_limits.y = y_coordinate_of_reference_point_for_command_limits;
	rds_network_ros::Point2D p_msg;
	for (auto& cp : my_cpg.collision_points)
	{
		p_msg.x = cp.p.x;
		p_msg.y = cp.p.y;
		msg_to_gui.collision_points_on_robot.push_back(p_msg);
		p_msg.x = (cp.p + cp.p_to_q).x;
		p_msg.y = (cp.p + cp.p_to_q).y;
		msg_to_gui.collision_points_on_obstacles.push_back(p_msg);
	}
	msg_to_gui.limit_velocity_command_abs_angular_at_zero_linear = hexagon_limits.absolute_angular_at_zero_linear;
	msg_to_gui.limit_velocity_command_max_linear = hexagon_limits.max_linear;
	rds_network_ros::Circle c_msg;
	for (auto& c : my_cpg.robot_shape_circles)
	{
		c_msg.center.x = c.center.x;
		c_msg.center.y = c.center.y;
		c_msg.radius = c.radius;
		msg_to_gui.robot_shape.push_back(c_msg);
	}
	publisher_for_gui.publish(msg_to_gui);

	return true;
}

RDSHolonomicNode::RDSHolonomicNode(ros::NodeHandle* n)
	: my_cpg()
	, laserscan_subscriber(n->subscribe<sensor_msgs::LaserScan>("laserscan", 1,
		&MyCollisionPointGenerator::obstacleMessageCallback, &my_cpg))
	, publisher_for_gui(n->advertise<rds_network_ros::ToGui>("rds_to_gui", 1)) 
	, command_correction_server(n->advertiseService("rds_velocity_command_correction",
		&RDSHolonomicNode::commandCorrectionService, this))
{
	ros::spin();
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "rds_holonomic_ros_node");
	ros::NodeHandle n;
	RDSHolonomicNode rds_node(&n);
	return 0;
}
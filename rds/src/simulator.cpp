#include "simulator.hpp"
#include "rds_wrap.hpp"
#include "orca_style.hpp"

#include <cmath>

using Geometry2D::Vec2;
using AdditionalPrimitives2D::Circle;

namespace RDS
{
	void Simulator::stepEuler(float dt)
	{
		if (use_orca_style)
		{
			orca_velocities.resize(obstacles.size());
			for (int i = 0; i < obstacles.size(); i++)
				orca_velocities[i] = OrcaStyle::avoid(robot, obstacles, i, time, tau_orca_style);
		}

		// set RDS parameters
		float abs_linear_acceleration_limit = 1.f;
		float abs_angular_acceleration_limit = 1.f;

		RDS::VelocityCommandHexagonLimits hexagon_limits;
		hexagon_limits.min_linear = -0.75f;
		hexagon_limits.max_linear = 1.75f;
		hexagon_limits.absolute_angular_at_min_linear = 0.f;
		hexagon_limits.absolute_angular_at_max_linear = 0.f;
		hexagon_limits.absolute_angular_at_zero_linear = 1.f;

		float y_coordinate_of_reference_point_for_command_limits = 0.5f;
		float weight_scaling_of_reference_point_for_command_limits = 1.f;
		//float tau = 2.f; is a member now 
		float delta = 0.05f;
		float clearance_from_axle_of_final_reference_point = 0.15f;

		// compute RDS command correction
		VelocityCommand nominal_command(robot.motion_law(time, robot.position, robot.orientation));
		
		RDS::VelocityCommandBoxLimits box_limits;
		box_limits.min_linear = robot.previous_command.linear - dt*abs_linear_acceleration_limit;
		box_limits.min_angular = robot.previous_command.angular - dt*abs_angular_acceleration_limit;
		box_limits.max_linear = robot.previous_command.linear + dt*abs_linear_acceleration_limit;
		box_limits.max_angular = robot.previous_command.angular + dt*abs_angular_acceleration_limit;

		std::vector<CollisionPoint> collision_points;
		createCollisionPointsInRobotFrame(&collision_points);

		RDSWrap rds_wrap(
			nominal_command,
			box_limits,
			hexagon_limits,
			collision_points,
			y_coordinate_of_reference_point_for_command_limits,
			weight_scaling_of_reference_point_for_command_limits,
			tau,
			delta,
			clearance_from_axle_of_final_reference_point,
			unilateral_velocity_shift,
			y_coordinate_of_reference_biasing_point,
			weight_of_reference_biasing_point);

		// for computing the acceleration-based constraints in the next step
		robot.previous_command = rds_wrap.getCommandSolution();
		// for display
		robot.previous_reference_point = rds_wrap.getReferencePoint();

		// perform the Euler integration step
		Vec2 robot_local_velocity = rds_wrap.getCommandSolution().pointVelocity(Vec2(0.f, 0.f));
		float rxx = std::cos(robot.orientation);
		float rxy = std::sin(robot.orientation);
		float ryx = -rxy;
		float ryy = rxx;
		Vec2 robot_global_velocity(rxx*robot_local_velocity.x + ryx*robot_local_velocity.y,
			rxy*robot_local_velocity.x + ryy*robot_local_velocity.y);
		robot.position = robot.position + dt*robot_global_velocity;
		robot.orientation += dt*rds_wrap.getCommandSolution().angular;
		if (use_orca_style)
		{
			for (int i = 0; i < obstacles.size(); i++)
				obstacles[i].position = obstacles[i].position + dt*orca_velocities[i];
		}
		else
		{
			for (auto& ob : obstacles)
			{
				if (ob.use_constant_motion_law)
					ob.position = ob.position + dt*ob.constant_motion_law;
				else
					ob.position = ob.position + dt*ob.motion_law(time, ob.position);
			}
		}
		time += dt;
	}

	void Simulator::createCollisionPointsInRobotFrame(std::vector<CollisionPoint>* collision_points) const
	{
		float rxx = std::cos(-robot.orientation);
		float rxy = std::sin(-robot.orientation);
		float ryx = -rxy;
		float ryy = rxx;

		std::vector<CollisionPoint>& cps = *collision_points;
		cps.resize(obstacles.size()*robot.shape.size());
		std::vector<CollisionPoint>::size_type i = 0;
		int j = 0;
		for (auto& ob : obstacles)
		{
			for (auto& rs : robot.shape)
			{
				Vec2 ob_velocity_global(ob.motion_law(time, ob.position));
				if (use_orca_style)
					ob_velocity_global = orca_velocities[j];
				else if (ob.use_constant_motion_law)
					ob_velocity_global = ob.constant_motion_law;
				Vec2 ob_velocity_local(rxx*ob_velocity_global.x + ryx*ob_velocity_global.y,
					rxy*ob_velocity_global.x + ryy*ob_velocity_global.y);
				if (!use_velocities)
					ob_velocity_local = Vec2(0.f, 0.f);
				Vec2 ob_position_diff = ob.position - robot.position;
				Vec2 ob_center_local = Vec2(rxx*ob_position_diff.x + ryx*ob_position_diff.y,
					rxy*ob_position_diff.x + ryy*ob_position_diff.y);
				cps[i] = CollisionPoint(rs, Circle(ob_center_local, ob.radius), ob_velocity_local);
				i++;
			}
			j++;
		}
	}
}
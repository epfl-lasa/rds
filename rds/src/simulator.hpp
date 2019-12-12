#ifndef SIMULATOR_HPP
#define SIMULATOR_HPP

#include "geometry.hpp"
#include "collision_point.hpp"
#include "differential_drive_kinematics.hpp"

#include <vector>

namespace RDS
{
	struct Simulator
	{
		typedef Geometry2D::Vec2 (*pointDS)(float time, const Geometry2D::Vec2& position);
		typedef VelocityCommand (*robotDS)(float time, const Geometry2D::Vec2& position, float orientation);

		struct Obstacle
		{
			Obstacle(pointDS motion_law, const Geometry2D::Vec2& initial_position, float radius,
				bool use_constant_motion_law = false, const Geometry2D::Vec2& constant_motion_law = Geometry2D::Vec2())
				: motion_law(motion_law)
				, position(initial_position)
				, radius(radius)
				, use_constant_motion_law(use_constant_motion_law)
				, constant_motion_law(constant_motion_law)
			{ }
			pointDS motion_law;
			Geometry2D::Vec2 position;
			float radius;
			bool use_constant_motion_law;
			Geometry2D::Vec2 constant_motion_law;
		};

		struct Robot
		{
			Robot(robotDS motion_law, const std::vector<AdditionalPrimitives2D::Circle>& shape,
				const Geometry2D::Vec2& initial_position, float initial_orientation, const VelocityCommand& previous_command)
				: motion_law(motion_law)
				, shape(shape)
				, position(initial_position)
				, orientation(initial_orientation)
				, previous_command(previous_command)
			{ }
			robotDS motion_law;
			std::vector<AdditionalPrimitives2D::Circle> shape;
			Geometry2D::Vec2 position;
			float orientation;
			VelocityCommand previous_command;
		};

		Simulator(robotDS robot_motion_law, const std::vector<AdditionalPrimitives2D::Circle>& robot_shape,
				const Geometry2D::Vec2& robot_initial_position, float robot_initial_orientation, const VelocityCommand& robot_previous_command,
				float tau = 2.f, bool unilateral_velocity_shift = false)
			: time(0.f)
			, robot(robot_motion_law, robot_shape, robot_initial_position, robot_initial_orientation, robot_previous_command)
			, use_orca_style(false)
			, tau(tau)
			, tau_orca_style(2.f)
			, unilateral_velocity_shift(unilateral_velocity_shift)
			, use_velocities(true)
			, weight_of_reference_biasing_point(0.f)
			, y_coordinate_of_reference_biasing_point(1.f)
		{ }

		void stepEuler(float dt);

		float time;
		Robot robot;
		std::vector<Obstacle> obstacles;
		std::vector<Geometry2D::Vec2> orca_velocities;
		bool use_orca_style;
		float tau;
		float tau_orca_style;
		bool unilateral_velocity_shift;
		bool use_velocities;
		float weight_of_reference_biasing_point;
		float y_coordinate_of_reference_biasing_point;


	private:
		void createCollisionPointsInRobotFrame(std::vector<CollisionPoint>* collision_points) const;
	};
}

#endif
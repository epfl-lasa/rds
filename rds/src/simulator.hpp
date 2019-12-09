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
			Obstacle(pointDS motion_law, const Geometry2D::Vec2& initial_position, float radius)
				: motion_law(motion_law)
				, position(initial_position)
				, radius(radius)
			{ }
			pointDS motion_law;
			Geometry2D::Vec2 position;
			float radius;
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
				const Geometry2D::Vec2& robot_initial_position, float robot_initial_orientation, const VelocityCommand& robot_previous_command)
			: robot(robot_motion_law, robot_shape, robot_initial_position, robot_initial_orientation, robot_previous_command)
		{ }

		void stepEuler(float dt);

		float time;
		Robot robot;
		std::vector<Obstacle> obstacles;

	private:
		void createCollisionPointsInRobotFrame(std::vector<CollisionPoint>* collision_points) const;
	};
}

#endif
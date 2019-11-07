#ifndef RDS_HPP
#define RDS_HPP

#include "geometry.hpp"

#include <vector>

namespace RDS
{
	class CollisionPoint
	{
	public:
		Geometry2D::Vec2 p; // point on the robot
		Geometry2D::Vec2 p_to_q; // vector connecting p to its closest point q on a given obstacle
		Geometry2D::Vec2 v_q;	// q's velocity
	};

	class CollisionPointGenerator
	{
	public:
		virtual ~CollisionPointGenerator() { };

		virtual const std::vector<CollisionPoint>& getCollisionPoints() const = 0;

		class CollisionException { };

	protected:
		void defineRobotShape(std::vector<AdditionalPrimitives2D::Circle>* robot_shape) const;
	};

	class CircleCollisionPointGenerator : public CollisionPointGenerator
	{
	public:
		CircleCollisionPointGenerator(const std::vector<AdditionalPrimitives2D::Circle>& obstacles,
			const std::vector<Geometry2D::Vec2>& obstacle_velocities);

		virtual const std::vector<CollisionPoint>& getCollisionPoints() const
		{
			return collision_points;
		}
	
	private:
		std::vector<CollisionPoint> collision_points;
	};

	class LRFCollisionPointGenerator : public CollisionPointGenerator
	{
	public:
		LRFCollisionPointGenerator(const std::vector<float>& range_scan, float angular_step_in_rad, 
			float scan_start_angle_in_rad, float min_range, float max_range);

		virtual const std::vector<CollisionPoint>& getCollisionPoints() const
		{
			return collision_points;
		}
	
	private:
		std::vector<CollisionPoint> collision_points;
	};

	class VelocityCommand
	{
	public:
		VelocityCommand()
			: linear(0.f)
			, angular(0.f)
		{ }

		VelocityCommand(float linear, float angular)
			: linear(linear)
			, angular(angular)
		{ }	

		float linear, angular;
	};

	class ConstraintGenerator
	{
	public:
		ConstraintGenerator(const VelocityCommand& nominal_command,
			const std::vector<CollisionPoint>& collision_points, const VelocityCommand& previous_command);

		const std::vector<Geometry2D::HalfPlane2>& getConstraints() const
		{
			return constraints;
		}

		VelocityCommand optimalSolutionToVelocityCommand(const Geometry2D::Vec2& optimal_solution) const
		{
			return pointVelocityToCommand(optimal_solution/scaling + reference_point_nominal_velocity, reference_point);
		}

		const Geometry2D::Vec2 reference_point;
		const Geometry2D::Vec2 reference_point_nominal_velocity;
	private:
		// velocity command limit corners generated for the reference point velocity
		const Geometry2D::Vec2 zero_linear_max_angular;
		const Geometry2D::Vec2 zero_linear_min_angular;
		const Geometry2D::Vec2 min_linear_max_angular;
		const Geometry2D::Vec2 min_linear_min_angular;
		const Geometry2D::Vec2 max_linear_max_angular;
		const Geometry2D::Vec2 max_linear_min_angular;
	public:
		const float scaling;

		static const float control_cycle_duration;
		// define limits for velocity command increments per unit cycle duration (i.e. accelerations)
		static const float linear_acceleration_limit;
		static const float angular_acceleration_limit;
		// define limits for velocity commands
		static const float minimum_linear_velocity;
		static const float maximum_linear_velocity;
		static const float angular_velocity_limit_at_zero_linear_velocity;
		static const float angular_velocity_limit_at_minimum_linear_velocity;
		static const float angular_velocity_limit_at_maximum_linear_velocity;

	private:
		Geometry2D::Vec2 computeReferencePoint(const VelocityCommand& nominal_command,
			const std::vector<CollisionPoint>& collision_points);
		float computeScaling();

		void generateConstraints(const std::vector<CollisionPoint>& collision_points,
			const VelocityCommand& previous_command);
		void appendConstraintsForVelocityLimits();
		void appendConstraintsForAccelerationLimits(const VelocityCommand& previous_command);
		void appendConstraintsForCollisionPoints(const std::vector<CollisionPoint>& collision_points);

		static void writeCollisionPointVelocityConstraint(const CollisionPoint& collision_point,
			Geometry2D::Vec2* normal, float* offset);

		static Geometry2D::Vec2 commandToPointVelocity(const VelocityCommand& command,
			const Geometry2D::Vec2& point)
		{
			return Geometry2D::Vec2(-command.angular*point.y, command.linear + command.angular*point.x);
		}

		static Geometry2D::Vec2 commandToPointVelocity(float linear, float angular,
			const Geometry2D::Vec2& point)
		{
			return Geometry2D::Vec2(-angular*point.y, linear + angular*point.x);
		}

		static VelocityCommand pointVelocityToCommand(const Geometry2D::Vec2& point_velocity,
			const Geometry2D::Vec2& point)
		{
			return VelocityCommand(point.x/point.y*point_velocity.x + point_velocity.y, -point_velocity.x/point.y);
		}

		std::vector<Geometry2D::HalfPlane2> constraints;
	};

	class CommandGenerator
	{
	public:
		CommandGenerator(const VelocityCommand& nominal_command, const VelocityCommand& previous_command,
			const CollisionPointGenerator* collision_point_generator);

		const ConstraintGenerator constraint_generator;
		const Geometry2D::Vec2 solution_distance_minimization;
		const VelocityCommand command;
	};
}

#endif
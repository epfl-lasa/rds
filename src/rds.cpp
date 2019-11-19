#include "rds.hpp"

#include "distance_minimizer.hpp"

#include <cmath>

#include <iostream>

namespace RDS
{
	void CollisionPointGenerator::defineRobotShape(std::vector<AdditionalPrimitives2D::Circle>* robot_shape) const
	{
		robot_shape->resize(0);

		// create a Qolo-like shape using 8 circles
		float scale = 0.3f*2.f/357.0f*2.f;

		Geometry2D::Vec2 position;
		float radius;

		position.x = 208.391*scale;
		position.y = 95.028*scale;
		radius = 77.637*scale;
		robot_shape->push_back(AdditionalPrimitives2D::Circle(position, radius));
		position.x = -208.391*scale;
		robot_shape->push_back(AdditionalPrimitives2D::Circle(position, radius));

		position.x = 153.585*scale;
		position.y = 4.019*scale;
		radius = 136.843*scale;
		robot_shape->push_back(AdditionalPrimitives2D::Circle(position, radius));
		position.x = -153.585*scale;
		robot_shape->push_back(AdditionalPrimitives2D::Circle(position, radius));

		position.x = 57.833*scale;
		position.y = -105.522*scale;
		radius = 227.437*scale;
		robot_shape->push_back(AdditionalPrimitives2D::Circle(position, radius));
		position.x = -57.833*scale;
		robot_shape->push_back(AdditionalPrimitives2D::Circle(position, radius));

		position.x = 0.0;
		position.y = -284.338*scale;
		radius = 230.051*scale;
		robot_shape->push_back(AdditionalPrimitives2D::Circle(position, radius));

		position.x = 0.0;
		position.y = -383.821*scale;
		radius = 174.055*scale;
		robot_shape->push_back(AdditionalPrimitives2D::Circle(position, radius));
	}

	CircleCollisionPointGenerator::CircleCollisionPointGenerator(const std::vector<AdditionalPrimitives2D::Circle>& obstacles,
			const std::vector<Geometry2D::Vec2>& obstacle_velocities)
	{
		std::vector<AdditionalPrimitives2D::Circle> robot_shape;
		this->CollisionPointGenerator::defineRobotShape(&robot_shape);

		CollisionPoint cp;
		for (std::vector<AdditionalPrimitives2D::Circle>::size_type i = 0; i != obstacles.size(); i++)
		{
			if (i < obstacle_velocities.size())
				cp.v_q = obstacle_velocities[i];
			else
				cp.v_q = Geometry2D::Vec2(0.f, 0.f);

			for (std::vector<AdditionalPrimitives2D::Circle>::size_type j = 0; j != robot_shape.size(); j++)
			{
				Geometry2D::Vec2 n_p_to_q((obstacles[i].center - robot_shape[j].center).normalized());
				cp.p = robot_shape[j].center + n_p_to_q*robot_shape[j].radius;
				float d = (obstacles[i].center - robot_shape[j].center).norm() - robot_shape[j].radius -
					obstacles[i].radius;
				if (d < 0.f)
					throw CollisionException();
				else
					cp.p_to_q =  n_p_to_q*d;
				collision_points.push_back(cp);
			}
		}
	}

	LRFCollisionPointGenerator::LRFCollisionPointGenerator(const std::vector<float>& range_scan,
		float angular_step_in_rad, float scan_start_angle_in_rad, float min_range, float max_range)
		: lrf_location(0.f, 0.f)
		, lrf_orientation(0.f)
	{
		min_range = 2.f;

		std::vector<Geometry2D::Vec2> scan_points;//(range_scan.size());
		for (std::vector<float>::size_type i = 0; i != range_scan.size(); i++)
		{
			float phi = lrf_orientation + scan_start_angle_in_rad + i*angular_step_in_rad;
			if ((range_scan[i] > min_range) && (range_scan[i] < max_range))
				scan_points.push_back(lrf_location + 
					range_scan[i]*Geometry2D::Vec2(std::cos(phi), std::sin(phi)));
		}

		std::vector<AdditionalPrimitives2D::Circle> robot_shape;
		this->CollisionPointGenerator::defineRobotShape(&robot_shape);

		collision_points.resize(robot_shape.size()*scan_points.size());

		CollisionPoint cp;
		cp.v_q = Geometry2D::Vec2(0.f, 0.f);
		for (std::vector<AdditionalPrimitives2D::Circle>::size_type i = 0; i != robot_shape.size(); i++)
		{
			for (std::vector<float>::size_type j = 0; j != scan_points.size(); j++)
			{
				Geometry2D::Vec2 n_p_to_q((scan_points[j] - robot_shape[i].center).normalized());
				cp.p = robot_shape[i].center + n_p_to_q*robot_shape[i].radius;
				float d = (scan_points[j] - robot_shape[i].center).norm() - robot_shape[i].radius;
				if (d < 0.f)
					throw CollisionException();
				else
					cp.p_to_q =  n_p_to_q*d;
				collision_points[i*scan_points.size() + j] = cp;
			}
		}
	}

	const float ConstraintGenerator::control_cycle_duration = 0.05f;//0.005f;

	const float ConstraintGenerator::linear_acceleration_limit = 3.f;
	const float ConstraintGenerator::angular_acceleration_limit = 6.282f/12.f;

	const float ConstraintGenerator::minimum_linear_velocity = -0.5f;
	const float ConstraintGenerator::maximum_linear_velocity = 1.5f;
	const float ConstraintGenerator::angular_velocity_limit_at_zero_linear_velocity = 6.282f/3.f;
	const float ConstraintGenerator::angular_velocity_limit_at_minimum_linear_velocity = 0.f;
	const float ConstraintGenerator::angular_velocity_limit_at_maximum_linear_velocity = 0.f;

	const float ConstraintGenerator::y_coordinate_of_reference_point_for_command_limits = 0.5f;
	const float ConstraintGenerator::weight_scaling_of_reference_point_for_command_limits = 1.0f;

	float ConstraintGenerator::computeScaling()
	{
		// for the six point velocities created by the corners of the limits for the velocity command,
		// compute the maximum absolute coordinate after subtracting the nominal reference point velocity
		float largest_absolute_coordinate = 0.f;
		float coordinates[12] = 
		{
			zero_linear_max_angular.x - reference_point_nominal_velocity.x,
			zero_linear_max_angular.y - reference_point_nominal_velocity.y,
			zero_linear_min_angular.x - reference_point_nominal_velocity.x,
			zero_linear_min_angular.y - reference_point_nominal_velocity.y,
			min_linear_max_angular.x - reference_point_nominal_velocity.x,
			min_linear_max_angular.y - reference_point_nominal_velocity.y,
			min_linear_min_angular.x - reference_point_nominal_velocity.x,
			min_linear_min_angular.y - reference_point_nominal_velocity.y,
			max_linear_max_angular.x - reference_point_nominal_velocity.x,
			max_linear_max_angular.y - reference_point_nominal_velocity.y,
			max_linear_min_angular.x - reference_point_nominal_velocity.x,
			max_linear_min_angular.y - reference_point_nominal_velocity.y
		};

		for (int i = 0; i < 12; i++)
		{
			if (std::abs(coordinates[i]) > largest_absolute_coordinate)
				largest_absolute_coordinate = std::abs(coordinates[i]);
		}

		return 0.5f/(largest_absolute_coordinate + 0.01f);
	}

	Geometry2D::Vec2 ConstraintGenerator::computeReferencePoint(const VelocityCommand& nominal_command,
		const std::vector<CollisionPoint>& collision_points)
	{
		Geometry2D::Vec2 reference_point(0.f, 0.f);
		float unnormalized_weight_sum = 0.f;

		// contribution by the reference point for command limits
		// ... annoying ...
		//Geometry2D::Vec2 v_lref_nominal = commandToPointVelocity(nominal_command, Geometry2D::Vec2(0.f,
		//	y_coordinate_of_reference_point_for_command_limits));

		//y_coordinate_of_reference_point_for_command_limits
		//weight_scaling_of_reference_point_for_command_limits

		// contribution by collision points
		Geometry2D::Vec2 normal;
		float offset;
		for (std::vector<CollisionPoint>::size_type i = 0; i != collision_points.size(); i++)
		{
			writeCollisionPointVelocityConstraint(collision_points[i], &normal, &offset);
			Geometry2D::Vec2 v_p_nominal(commandToPointVelocity(nominal_command, collision_points[i].p));
			float violation = v_p_nominal.dot(normal) - offset;
			if (violation > 0.f)
			{
				reference_point = reference_point + violation*collision_points[i].p;
				unnormalized_weight_sum += violation;
			}
		}
		reference_point = reference_point/(unnormalized_weight_sum + 0.0001f);

		// mirror and shift the reference point to lie in front of the axle
		reference_point.y = std::abs(reference_point.y);
		if (reference_point.y < 0.15f)
			reference_point.y = 0.15f;

		return reference_point;
	}

	ConstraintGenerator::ConstraintGenerator(const VelocityCommand& nominal_command,
		const std::vector<CollisionPoint>& collision_points, const VelocityCommand& previous_command)
		: reference_point(computeReferencePoint(nominal_command, collision_points))
		, reference_point_nominal_velocity(commandToPointVelocity(nominal_command, reference_point))
		, zero_linear_max_angular(commandToPointVelocity(0.f,
			angular_velocity_limit_at_zero_linear_velocity,
			reference_point))
		, zero_linear_min_angular(commandToPointVelocity(0.f,
			-angular_velocity_limit_at_zero_linear_velocity,
			reference_point))
		, min_linear_max_angular(commandToPointVelocity(minimum_linear_velocity,
			angular_velocity_limit_at_minimum_linear_velocity,
			reference_point))
		, min_linear_min_angular(commandToPointVelocity(minimum_linear_velocity,
			-angular_velocity_limit_at_minimum_linear_velocity,
			reference_point))
		, max_linear_max_angular(commandToPointVelocity(maximum_linear_velocity,
			angular_velocity_limit_at_maximum_linear_velocity,
			reference_point))
		, max_linear_min_angular(commandToPointVelocity(maximum_linear_velocity,
			-angular_velocity_limit_at_maximum_linear_velocity,
			reference_point))
		, scaling(computeScaling())
	{
		generateConstraints(collision_points, previous_command);
	}

	void ConstraintGenerator::generateConstraints(const std::vector<CollisionPoint>& collision_points,
		const VelocityCommand& previous_command)
	{
		appendConstraintsForVelocityLimits();
		appendConstraintsForAccelerationLimits(previous_command);
		appendConstraintsForCollisionPoints(collision_points);
	}

	Geometry2D::Vec2 rot90Clockwise(const Geometry2D::Vec2& v)
	{
		return Geometry2D::Vec2(v.y, -v.x);
	}

	void computeHalfplaneContainingPointCFromPointsAB(const Geometry2D::Vec2& pa,
		const Geometry2D::Vec2& pb, const Geometry2D::Vec2& pc, Geometry2D::Vec2* normal, float* offset)
	{
		if (rot90Clockwise(pa - pb).normalized().dot(pa - pc) > 0.f)
		{
			*offset = rot90Clockwise(pa - pb).normalized().dot(pa);
			*normal = rot90Clockwise(pa - pb).normalized();
		}
		else
		{
			*offset = rot90Clockwise(pa - pb).normalized().dot(pa)*(-1.f);
			*normal = rot90Clockwise(pa - pb).normalized()*(-1.f);
		}
	}

	void computeHalfplaneContainingTheOriginFromTwoBoundaryPoints(const Geometry2D::Vec2& p1,
		const Geometry2D::Vec2& p2, Geometry2D::Vec2* normal, float* offset)
	{
		computeHalfplaneContainingPointCFromPointsAB(p1, p2, Geometry2D::Vec2(0.f, 0.f), normal, offset);
		/*
		if (rot90Clockwise(p1 - p2).normalized().dot(p1) > 0.f)
		{
			*offset = rot90Clockwise(p1 - p2).normalized().dot(p1);
			*normal = rot90Clockwise(p1 - p2).normalized();
		}
		else
		{
			*offset = rot90Clockwise(p1 - p2).normalized().dot(p1)*(-1.f);
			*normal = rot90Clockwise(p1 - p2).normalized()*(-1.f);
		}
		*/
	}

	void ConstraintGenerator::appendConstraintsForVelocityLimits()
	{
		Geometry2D::Vec2 normal; // compute halfplane normals assuming that (0,0) is within the limits
		float offset;

		computeHalfplaneContainingTheOriginFromTwoBoundaryPoints(zero_linear_max_angular,
			max_linear_max_angular, &normal, &offset);
		constraints.push_back(Geometry2D::HalfPlane2(normal,
			scaling*(offset - normal.dot(reference_point_nominal_velocity))));

		computeHalfplaneContainingTheOriginFromTwoBoundaryPoints(zero_linear_max_angular,
			min_linear_max_angular, &normal, &offset);
		constraints.push_back(Geometry2D::HalfPlane2(normal,
			scaling*(offset - normal.dot(reference_point_nominal_velocity))));

		computeHalfplaneContainingTheOriginFromTwoBoundaryPoints(zero_linear_min_angular,
			max_linear_min_angular, &normal, &offset);
		constraints.push_back(Geometry2D::HalfPlane2(normal,
			scaling*(offset - normal.dot(reference_point_nominal_velocity))));

		computeHalfplaneContainingTheOriginFromTwoBoundaryPoints(zero_linear_min_angular,
			min_linear_min_angular, &normal, &offset);
		constraints.push_back(Geometry2D::HalfPlane2(normal,
			scaling*(offset - normal.dot(reference_point_nominal_velocity))));

		if (angular_velocity_limit_at_minimum_linear_velocity > 0.f)
		{
			computeHalfplaneContainingTheOriginFromTwoBoundaryPoints(min_linear_min_angular,
				min_linear_max_angular, &normal, &offset);
			constraints.push_back(Geometry2D::HalfPlane2(normal,
				scaling*(offset - normal.dot(reference_point_nominal_velocity))));
		}

		if (angular_velocity_limit_at_maximum_linear_velocity > 0.f)
		{
			computeHalfplaneContainingTheOriginFromTwoBoundaryPoints(max_linear_min_angular,
				max_linear_max_angular, &normal, &offset);
			constraints.push_back(Geometry2D::HalfPlane2(normal,
				scaling*(offset - normal.dot(reference_point_nominal_velocity))));
		}
	}

	void ConstraintGenerator::appendConstraintsForAccelerationLimits(const VelocityCommand& previous_command)
	{
		float max_angular = previous_command.angular + control_cycle_duration*angular_acceleration_limit;
		float min_angular = previous_command.angular - control_cycle_duration*angular_acceleration_limit;
		float max_linear = previous_command.linear + control_cycle_duration*linear_acceleration_limit;
		float min_linear = previous_command.linear - control_cycle_duration*linear_acceleration_limit;
		Geometry2D::Vec2 acc_max_linear_max_angular(commandToPointVelocity(max_linear, max_angular, reference_point));
		Geometry2D::Vec2 acc_max_linear_min_angular(commandToPointVelocity(max_linear, min_angular, reference_point));
		Geometry2D::Vec2 acc_min_linear_max_angular(commandToPointVelocity(min_linear, max_angular, reference_point));
		Geometry2D::Vec2 acc_min_linear_min_angular(commandToPointVelocity(min_linear, min_angular, reference_point));

		//std::cout << acc_max_linear_max_angular.x << ", " << acc_max_linear_max_angular.y << std::endl;

		Geometry2D::Vec2 previous_reference_point_velocity(commandToPointVelocity(previous_command, reference_point));

		//std::cout << "reference point velocity with previous command: " << previous_reference_point_velocity.x << ", " << previous_reference_point_velocity.y << std::endl;

		Geometry2D::Vec2 normal; // compute halfplane normals assuming that previous_reference_point_velocity is within the limits
		float offset;

		computeHalfplaneContainingPointCFromPointsAB(acc_max_linear_max_angular, acc_max_linear_min_angular,
			previous_reference_point_velocity, &normal, &offset);
		constraints.push_back(Geometry2D::HalfPlane2(normal,
			scaling*(offset - normal.dot(reference_point_nominal_velocity))));

		//std::cout << Geometry2D::HalfPlane2(normal, offset).signedDistance(previous_reference_point_velocity) << std::endl;
		//std::cout << constraints.back().signedDistance(scaling*(previous_reference_point_velocity-reference_point_nominal_velocity)) << std::endl;

		computeHalfplaneContainingPointCFromPointsAB(acc_min_linear_max_angular, acc_min_linear_min_angular,
			previous_reference_point_velocity, &normal, &offset);
		constraints.push_back(Geometry2D::HalfPlane2(normal,
			scaling*(offset - normal.dot(reference_point_nominal_velocity))));

		computeHalfplaneContainingPointCFromPointsAB(acc_min_linear_min_angular, acc_max_linear_min_angular,
			previous_reference_point_velocity, &normal, &offset);
		constraints.push_back(Geometry2D::HalfPlane2(normal,
			scaling*(offset - normal.dot(reference_point_nominal_velocity))));

		computeHalfplaneContainingPointCFromPointsAB(acc_min_linear_max_angular, acc_max_linear_max_angular,
			previous_reference_point_velocity, &normal, &offset);
		constraints.push_back(Geometry2D::HalfPlane2(normal,
			scaling*(offset - normal.dot(reference_point_nominal_velocity))));
	}

	void ConstraintGenerator::writeCollisionPointVelocityConstraint(const CollisionPoint& collision_point,
		Geometry2D::Vec2* normal, float* offset)
	{
		float d = collision_point.p_to_q.norm();
		*normal = collision_point.p_to_q.normalized();
		*offset = 1.f/2.f*(d - 0.1f) + collision_point.v_q.dot(*normal);
	}

	void transformPointAToPointBConstraint(const Geometry2D::Vec2& p_a, const Geometry2D::Vec2& p_b,
		const Geometry2D::Vec2& normal_a, float offset_a, Geometry2D::Vec2* normal_b, float* offset_b)
	{
		// B_to_A^T = [y_A/y_B, (x_B-x_A)/y_B; 0 1]
		*normal_b = Geometry2D::Vec2(normal_a.x*p_a.y/p_b.y + normal_a.y*(p_b.x-p_a.x)/p_b.y,
			normal_a.y);
		float norm = normal_b->norm();
		*normal_b = normal_b->normalized(); // use it because it throws an exception if the norm is zero
		*offset_b = offset_a/norm;
	}

	void ConstraintGenerator::appendConstraintsForCollisionPoints(const std::vector<CollisionPoint>& collision_points)
	{
		Geometry2D::Vec2 normal_collision_point, normal_reference_point;
		float offset_collision_point, offset_reference_point;
		for (std::vector<CollisionPoint>::size_type i = 0; i != collision_points.size(); i++)
		{
			writeCollisionPointVelocityConstraint(collision_points[i], &normal_collision_point,
				&offset_collision_point);
			transformPointAToPointBConstraint(collision_points[i].p, reference_point,
				normal_collision_point, offset_collision_point,
				&normal_reference_point, &offset_reference_point);
			constraints.push_back(Geometry2D::HalfPlane2(normal_reference_point, scaling*(offset_reference_point
				- normal_reference_point.dot(reference_point_nominal_velocity))));
		}
	}

	CommandGenerator::CommandGenerator(const VelocityCommand& nominal_command,
		const VelocityCommand& previous_command, const CollisionPointGenerator* collision_point_generator)
		: constraint_generator(nominal_command, collision_point_generator->getCollisionPoints(), previous_command)
		, solution_distance_minimization(Geometry2D::DistanceMinimizer::IncrementalDistanceMinimization(
			constraint_generator.getConstraints()))
		, command(constraint_generator.optimalSolutionToVelocityCommand(solution_distance_minimization))
	{
		//std::cout << "Nominal command: " << nominal_command.linear << ", " << nominal_command.angular << std::endl;
		//std::cout << "Reference point: " << constraint_generator.reference_point.x << ", " << constraint_generator.reference_point.y << std::endl;
		//std::cout << "Reference point nominal v: " << constraint_generator.reference_point_nominal_velocity.x << ", " << constraint_generator.reference_point_nominal_velocity.y << std::endl;
		//std::cout << "solution_distance_minimization: " << solution_distance_minimization.x << ", " << solution_distance_minimization.y << std::endl;
		//std::cout << "Reference point rds velocity" << solution_distance_minimization.x/constraint_generator.scaling + constraint_generator.reference_point_nominal_velocity.x <<
		//	", " << solution_distance_minimization.y/constraint_generator.scaling + constraint_generator.reference_point_nominal_velocity.y << std::endl;
		//std::cout << "rds command: " << command.linear << ", " << command.angular << std::endl;
	}
}
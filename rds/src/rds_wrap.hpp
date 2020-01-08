#ifndef RDS_WRAP_HPP
#define RDS_WRAP_HPP

#include "rds_core.hpp"
#include "differential_drive_kinematics.hpp"
#include "collision_point.hpp"
#include "geometry.hpp"

#include <vector>
#include <iostream>

// making this a template just for fun
template <typename T>
struct CollisionPointGenerator
{
	// provide a constructor in the child class to define robot_shape_circles

	virtual ~CollisionPointGenerator() { };

	// define it to update the obstacle_circles and obstacle_velocities from messages of type T
	virtual void obstacleMessageCallback(const T& obstacle_sensor_msg) = 0;

	const CollisionPointGenerator& generateCollisionPoints()
	/*{
		collision_points.resize(0);//obstacle_circles.size()*robot_shape_circles.size());
		//collision_points[j*obstacle_circles.size() + i] = RDS::CollisionPoint(robot_shape_circles[j],
		//	obstacle_circles[i], obstacle_velocities[i]);
		for (std::vector<AdditionalPrimitives2D::Circle>::size_type i = 0; i != obstacle_circles.size(); i++)
		{
			try
			{
				for (std::vector<AdditionalPrimitives2D::Circle>::size_type j = 0; j != robot_shape_circles.size(); j++)
					RDS::CollisionPoint(robot_shape_circles[j], obstacle_circles[i], obstacle_velocities[i]);
				// no collision exception occurred, so create the collision points for this obstacle
				for (std::vector<AdditionalPrimitives2D::Circle>::size_type j = 0; j != robot_shape_circles.size(); j++)
				{
					collision_points.push_back(RDS::CollisionPoint(robot_shape_circles[j], obstacle_circles[i], obstacle_velocities[i]));
				}
			}
			catch (RDS::CollisionPoint::CollisionException e)
			{
				//std::cout << "Skipping colliding point ..." << std::endl;
			}
		}
		return *this;
	}*/
	{
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

	std::vector<AdditionalPrimitives2D::Circle> robot_shape_circles;
	std::vector<AdditionalPrimitives2D::Circle> obstacle_circles;
	std::vector<Geometry2D::Vec2> obstacle_velocities;
	std::vector<RDS::CollisionPoint> collision_points;
};

struct RDSWrap
{
	RDSWrap(const RDS::VelocityCommand& nominal_command,
		const RDS::VelocityCommandBoxLimits& box_limits,
		const RDS::VelocityCommandHexagonLimits& hexagon_limits,
		const std::vector<RDS::CollisionPoint>& collision_points,
		float y_coordinate_of_reference_point_for_command_limits = 0.5f,
		float weight_scaling_of_reference_point_for_command_limits = 1.f,
		float tau = 2.f,
		float delta = 0.1f,
		float clearance_from_axle_of_final_reference_point = 0.15f,
		bool unilateral_velocity_shift = false,
		float y_coordinate_of_reference_biasing_point = 1.f,
		float weight_of_reference_biasing_point = 0.f,
		bool use_exponential_weighting = false);

	const RDS::VelocityCommand& getCommandSolution() const
	{
		return rpvo.getCommandSolution();
	}

	const Geometry2D::Vec2& getReferencePoint() const
	{
		return rpg.getReferencePoint();
	}

	const Geometry2D::Vec2& getReferencePointVelocitySolution() const
	{
		return rpvo.getReferencePointVelocitySolution();
	}

	const std::vector<Geometry2D::HalfPlane2>& getReferencePointVelocityConstraints() const
	{
		return rpvcc.getConstraints();
	}

	const Geometry2D::Vec2& getScaledShiftedSolution() const
	{
		return rpvo.getScaledShiftedSolution();
	}

	const std::vector<Geometry2D::HalfPlane2>& getScaledShiftedConstraints() const
	{
		return rpvo.getScaledShiftedConstraints();
	}

	const std::vector<RDS::PointVelocityConstraint>& getPointVelocityCollisionConstraints() const
	{
		return pvcg.getCollisionConstraints();
	}

	bool isFeasible() const
	{
		return rpvo.isFeasible();
	}

private:
	RDS::PointVelocityConstraintGenerator pvcg;
	RDS::ReferencePointGenerator rpg;
	RDS::ReferencePointVelocityConstraintCompiler rpvcc;
	RDS::ReferencePointVelocityOptimization rpvo;
};

#endif

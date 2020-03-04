#ifndef RDS_4_HPP
#define RDS_4_HPP

#include "geometry.hpp"
#include "capsule.hpp"
#include <vector>

struct MovingCircle
{
	MovingCircle() { }
	MovingCircle(const AdditionalPrimitives2D::Circle& circle, const Geometry2D::Vec2& velocity)
	: circle(circle), velocity(velocity) { }

	AdditionalPrimitives2D::Circle circle;
	Geometry2D::Vec2 velocity;
};

namespace Geometry2D
{
	struct RDS4
	{
		RDS4(float tau, float delta, float v_max);

		void computeCorrectedVelocity(const Capsule& robot_shape, const Vec2& p_ref, const Vec2& v_nominal_p_ref,
			const std::vector<MovingCircle>& objects, Vec2* v_corrected_p_ref);

		float tau, delta, v_max, v_max_sqrt_2;

		std::vector<Geometry2D::HalfPlane2> constraints;

	private:
		void generateConstraints(const Capsule& robot_shape, const Vec2& p_ref, const Vec2& v_nominal_p_ref,
			const std::vector<MovingCircle>& objects, std::vector<HalfPlane2>* constraints);

		void generateAndAddConstraint(const Vec2& robot_point, float robot_radius,
			const Vec2& object_point, float object_radius, const Vec2& object_velocity,
			const Vec2& p_ref, const Vec2& v_nominal_p_ref, std::vector<HalfPlane2>* constraints);
		
		void solve(const Vec2& v_nominal, std::vector<HalfPlane2>& constraints, Vec2* v_corrected);
	};
}


#endif
#ifndef RDS_5_HPP
#define RDS_5_HPP

#include "geometry.hpp"
#include "capsule.hpp"
#include "vw_limits.hpp"
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
	struct RDS5
	{
		RDS5(float tau, float delta, float y_p_ref, const VWBox& vw_box_limits,
			const VWDiamond& vw_diamond_limits);

		void computeCorrectedVelocity(const Capsule& robot_shape,
			const Vec2& v_nominal_p_ref, const Vec2& v_previous_command,
			const std::vector<MovingCircle>& static_objects,
			const std::vector<MovingCircle>& moving_objects,
			Vec2* v_corrected_p_ref);

		const float tau, delta, y_p_ref;
		bool use_previous_command_as_basis, use_orca_style_crvo, use_conservative_shift;
		int n_bounding_circles;
		BoundingCircles bounding_circles;

		std::vector<Geometry2D::HalfPlane2> constraints_box_limits;
		std::vector<Geometry2D::HalfPlane2> constraints_diamond_limits;
		std::vector<Geometry2D::HalfPlane2> constraints;

	private:
		float v_box_x_min, v_box_x_max, v_box_y_min, v_box_y_max;
		float v_p_ref_radial_max;
		void generateVOConstraints(const Capsule& robot_shape, const Vec2& p_ref, const Vec2& v_nominal_p_ref,
			const std::vector<MovingCircle>& objects, std::vector<HalfPlane2>* constraints);

		void generateAndAddConstraint(const Vec2& robot_point, float robot_radius,
			const Vec2& object_point, float object_radius, const Vec2& object_velocity,
			const Vec2& p_ref, const Vec2& v_nominal_p_ref, std::vector<HalfPlane2>* constraints);
		
		void generateStaticConstraints(const Capsule& robot_shape, const Vec2& p_ref,
			const std::vector<MovingCircle>& static_objects, std::vector<HalfPlane2>* constraints);

		void generateAndAddStaticConstraint(const Vec2& robot_point,
			float robot_radius, const Vec2& object_point, float object_radius,
			const Vec2& p_ref, std::vector<HalfPlane2>* constraints);

		void solve(const Vec2& v_nominal, std::vector<HalfPlane2>& constraints, Vec2* v_corrected);
	};
}

#endif
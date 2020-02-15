#ifndef RDS_2_HPP
#define RDS_2_HPP

#include "geometry.hpp"
#include "capsule.hpp"
#include <vector>

namespace Geometry2D
{
	struct RDS2
	{
		RDS2(float T, float D, float delta, float v_max);

		void computeCorrectedVelocity(const Capsule& robot_shape, const Vec2& p_ref, const Vec2& v_nominal_p_ref,
			const std::vector<AdditionalPrimitives2D::Circle>& objects, Vec2* v_corrected_p_ref);

		float T, D, delta, v_max, v_max_sqrt_2;

		std::vector<Geometry2D::HalfPlane2> constraints;

	private:
		void generateConstraints(const Capsule& robot_shape, const Vec2& p_ref,
			const std::vector<AdditionalPrimitives2D::Circle>& objects,
			std::vector<HalfPlane2>* constraints);
		
		void solve(const Vec2& v_nominal, std::vector<HalfPlane2>& constraints, Vec2* v_corrected);
	};
}
#endif
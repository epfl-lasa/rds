#ifndef RSVO_HPP
#define RSVO_HPP

#include "geometry.hpp"
#include "capsule.hpp"

namespace Geometry2D
{
class RSVO
{
	int determineSweepingAngles(const Vec2& v_nominal_front_cap);

	void constructLegsOfRelativeVO();

	static int computeCircleTangents(const Vec2& center, float radius,
		Vec2* p_tangent_clockwise, Vec2* p_tangent_counter_clockwise);

	void constructConservativeTangentOfVO(const Vec2& v_object);

	enum struct LegType {front_cap, left_side, rear_left_cap, rear_swept_cap,
		rear_right_cap, right_side};

	float m_tau;
	float m_w_band_width;
	Capsule m_capsule;
	AdditionalPrimitives2D::Circle m_object;
	bool m_collision;
	float m_sweeping_angle_clockwise, m_sweeping_angle_counter_clockwise;
	HalfPlane2 leg_clockwise, leg_counter_clockwise;
	LegType leg_type_clockwise, leg_type_counter_clockwise;
	HalfPlane2 m_conservative_tangent_of_VO;

public:
	RSVO(float tau, float w_band_width, float delta, float y_front_cap, float y_rear_cap, float radius_capsule,
		const AdditionalPrimitives2D::Circle& object, const Vec2& v_nominal_front_cap,
		const Vec2& v_object)
	: m_tau(tau)
	, m_w_band_width(w_band_width)
	, m_capsule(radius_capsule + delta, Vec2(0.f, y_front_cap), Vec2(0.f, y_rear_cap))
	, m_object(object)
	, m_collision(false)
	{
		if (0 != determineSweepingAngles(v_nominal_front_cap))
		{
			m_collision = true;
			return;
		}
		constructLegsOfRelativeVO();
		constructConservativeTangentOfVO(v_object);
	}

	const HalfPlane2& getConservativeTangentOfVO() { return m_conservative_tangent_of_VO; }
};
}

#endif
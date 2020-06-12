#ifndef VW_LIMITS_HPP
#define VW_LIMITS_HPP

struct VWBox
{
	VWBox(float v_min, float v_max, float w_min, float w_max)
		: v_min(v_min), v_max(v_max), w_min(w_min), w_max(w_max) { }
	float v_min, v_max, w_min, w_max;
};

struct VWDiamond
{
	VWDiamond() : v_min(0.f), v_max(0.f), w_abs_max(0.f), v_at_w_abs_max(0.f) { }
	VWDiamond(float v_min, float v_max, float w_abs_max, float v_at_w_abs_max)
		: v_min(v_min), v_max(v_max), w_abs_max(w_abs_max), v_at_w_abs_max(v_at_w_abs_max) { }
	float v_min, v_max, w_abs_max, v_at_w_abs_max;
};

#endif
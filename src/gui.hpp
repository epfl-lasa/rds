#ifndef GUI_HPP
#define GUI_HPP

#include "geometry.hpp"
#include "window.hpp"

class GUI
{
public:
	// if minutes_to_run > 0, the GUI will display things for 
	// at least minutes_to_run [minutes] (generally a bit longer),
	// or if minutes_to_run <= 0 until one closes the window.
	GUI(float minutes_to_run,
		const std::vector<Geometry2D::HalfPlane2>& half_planes,
		const std::vector<Geometry2D::Vec2>& points);
	~GUI();

private:
	Window* window;
};

#endif

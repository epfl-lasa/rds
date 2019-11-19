#ifndef GUI_HPP
#define GUI_HPP

#include "geometry.hpp"
#include "window.hpp"
#include <vector>

/*
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
};*/



class GUI
{
public: 
	GUI(const char* title, float window_size_in_distance_units);

	~GUI();

	void blockingShowUntilClosed();

	int update();

	const std::vector<Geometry2D::HalfPlane2>* halfplanes;
	const std::vector<Geometry2D::Vec2>* points;
	const std::vector<AdditionalPrimitives2D::Circle>* circles;
	std::vector<Window::sdlColor> points_colors;
	const std::vector<AdditionalPrimitives2D::Arrow>* arrows;
	std::vector<Window::sdlColor> arrows_colors;
	//std::vector<Window::sdlColor> circles_colors;

private:
	Window* window;
};

#endif

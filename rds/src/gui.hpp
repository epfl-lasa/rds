#ifndef GUI_HPP
#define GUI_HPP

#include "geometry.hpp"
#include "capsule.hpp"
#include "window.hpp"
#include <vector>

typedef Window::sdlColor GuiColor;

class GUI
{
public: 
	GUI(const char* title, float window_size_in_distance_units, unsigned int window_size_in_pixels = 1000);

	~GUI();

	void blockingShowUntilClosed();

	int update();

	const std::vector<Geometry2D::HalfPlane2>* halfplanes;
	const std::vector<Geometry2D::Vec2>* points;
	const std::vector<AdditionalPrimitives2D::Circle>* circles;
	std::vector<GuiColor> points_colors;
	const std::vector<AdditionalPrimitives2D::Arrow>* arrows;
	std::vector<GuiColor> arrows_colors;
	std::vector<Window::sdlColor> circles_colors;
	const std::vector<AdditionalPrimitives2D::Polygon>* polygons;
	const std::vector<Geometry2D::Capsule>* capsules;


	void activateHalfplaneAreaRendering()
	{
		window->render_feasible_region = true;
	}

private:
	Window* window;
};

#endif

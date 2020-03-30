#include "gui.hpp"

using namespace AdditionalPrimitives2D;

GUI::GUI(const char* title, float window_size_in_distance_units, unsigned int window_size_in_pixels)
{
	halfplanes = 0;
	points = 0;
	circles = 0;
	arrows = 0;
	capsules = 0;
	polygons = 0;

	bool window_is_good;
	window = new Window(title, window_size_in_distance_units, window_size_in_pixels, 10.0, &window_is_good);
	if (!window_is_good)
	{
		delete window;
		window = 0;
	}
}


int GUI::update()
{
	if (window)
	{
		bool closed = window->render(halfplanes, points, points_colors, circles,
			arrows, arrows_colors, circles_colors, polygons, capsules);
		return closed ? 1 : 0;
	}
	else
		return 1;
}

void GUI::blockingShowUntilClosed()
{
	if (window)
	{
		while (!window->render(halfplanes, points, points_colors, circles,
			arrows, arrows_colors, circles_colors, polygons, capsules))
		{
			window->delayUntilNextFrameInMinutes();
		}
		// delete the window
		delete window;
		window = 0;
	}
}


/*
	bool window_is_good;
	window = new Window("GUI", 1.0, 1000, 10.0, &window_is_good);
	if (!window_is_good)
	{

		delete window;
		window = 0;
		return;
	}

	std::vector<Circle> circles(0); //{Circle(Geometry2D::Vec2(0,0), 0.5)};

	// give a green color to all the points
	std::vector<Window::sdlColor> points_colors;
	Window::sdlColor green;
	green.r = green.b = 0;
	green.g = 255;
	for (int i = 0; i < points.size(); i++)
		points_colors.push_back(green);

	if (minutes_to_run > 0.0)
	{
		// display the points and half-planes for minutes_to_run [minutes]
		float remaining_minutes = minutes_to_run;
		while (remaining_minutes > 0.0 &&
			!window->render(half_planes, points, points_colors, circles))
		{
			remaining_minutes -= window->delayUntilNextFrameInMinutes();
		}
	}
	else
	{
		// display the points and half-planes until the window is closed
		while (!window->render(half_planes, points, points_colors, circles))
		{
			window->delayUntilNextFrameInMinutes();
		}
	}

	// delete the window
	delete window;
	window = 0;
}*/

GUI::~GUI()
{
	if (window)
	{
		delete window;
		window = 0;
	}
}
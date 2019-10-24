#include "window.hpp"

#include <iostream>

std::vector<Window::SDLPointers> Window::allSdlPointers = std::vector<SDLPointers>(0);
bool Window::sdlIsInitialized = false;

Window::Window(const char* name, float screen_size_in_distance_units,
	int screen_size_in_pixels, float frame_rate_in_Hz, bool* is_good)
	:
	screenSizeInDistanceUnits(screen_size_in_distance_units),
	screenSizeInPixels(screen_size_in_pixels),
	pixelsPerDistanceUnit(screen_size_in_pixels/screen_size_in_distance_units),
	sdlWindowCreationNumber(allSdlPointers.size()),
	frameRateInHz(frame_rate_in_Hz)
{
	SDLPointers sdl_pointers;
	sdl_pointers.window = 0;
	sdl_pointers.renderer = 0;
	allSdlPointers.push_back(sdl_pointers);

	*is_good = false;
	if (!sdlIsInitialized && (SDL_Init(SDL_INIT_VIDEO) < 0))
	{
		std::cout << "Could not initialize SDL." << std::endl;
		return;
	}
	sdlIsInitialized = true;

	allSdlPointers[sdlWindowCreationNumber].window = SDL_CreateWindow(name, SDL_WINDOWPOS_UNDEFINED,
		SDL_WINDOWPOS_UNDEFINED, screenSizeInPixels, screenSizeInPixels, SDL_WINDOW_SHOWN);
	if (allSdlPointers[sdlWindowCreationNumber].window == 0)
	{
		std::cout << "Could not create window. SDL_Error: " << SDL_GetError() << std::endl;
		return;
	}

	allSdlPointers[sdlWindowCreationNumber].renderer = SDL_CreateRenderer(
		allSdlPointers[sdlWindowCreationNumber].window, -1, SDL_RENDERER_ACCELERATED);
	if (allSdlPointers[sdlWindowCreationNumber].renderer == 0)
	{
		std::cout << "Could not create renderer. SDL_Error: " << SDL_GetError() << std::endl;
		return;
	}
	*is_good = true;
}

Window::~Window()
{
	killSdlWindow(sdlWindowCreationNumber);

	// check if this is the last object which had a living window and quit sdl in that case
	bool all_dead = true;
	for (int i = 0; i < allSdlPointers.size(); i++)
	{
		if (allSdlPointers[i].window)
			all_dead = false;
	}
	if (all_dead)
	{
		sdlIsInitialized = false;
		SDL_Quit();
	}
}

void Window::killSdlWindow(int window_creation_number)
{
	if (allSdlPointers[window_creation_number].renderer)
	{
		SDL_DestroyRenderer(allSdlPointers[window_creation_number].renderer);
		allSdlPointers[window_creation_number].renderer = 0;
	}

	if (allSdlPointers[window_creation_number].window)
	{
		SDL_DestroyWindow(allSdlPointers[window_creation_number].window);
		allSdlPointers[window_creation_number].window = 0;
	}
}

bool Window::sdlCheck()
{
	// process events, kill windows if they have been closed
	SDL_Event e;
	while(SDL_PollEvent(&e) != 0)
	{
		if (e.type == SDL_QUIT) // kill all windows
		{
			for (int i = 0; i < allSdlPointers.size(); i++)
				killSdlWindow(i);
		}
		else if (e.type == SDL_WINDOWEVENT &&
			e.window.event == SDL_WINDOWEVENT_CLOSE) // kill the respective window
		{
			for (int i = 0; i < allSdlPointers.size(); i++)
			{
				if (SDL_GetWindowID(allSdlPointers[i].window) == e.window.windowID)
					killSdlWindow(i);
			}	
		}
	}
	// check if some object has killed this object's sdl window and return true/false
	if (allSdlPointers[sdlWindowCreationNumber].window == 0)
		return true;
	else
		return false;
}

float Window::delayUntilNextFrameInMinutes()
{
	SDL_Delay(1000.0/frameRateInHz);
	return 1.0/frameRateInHz/60.0;
}

bool Window::render(const std::vector<Geometry2D::HalfPlane2>& half_planes,
		const std::vector<Geometry2D::Vec2>& points,
		const std::vector<sdlColor>& points_colors,
		const std::vector<Geometry2D::Circle2>& circles)
{
	if (sdlCheck())
		return true;
	// for convenience
	SDL_Renderer* renderer = allSdlPointers[sdlWindowCreationNumber].renderer;
	// make the screen black
	SDL_SetRenderDrawColor(renderer, 0, 0, 0, 0);
	SDL_RenderClear(renderer);
	// for half planes
	renderHalfPlanes(half_planes);
	// for points
	float radius = 0.005*screenSizeInDistanceUnits;
	for (int i = 0; i < points.size(); i++)
	{
		if (i < points_colors.size())
		{
			SDL_SetRenderDrawColor(renderer, points_colors[i].r, points_colors[i].g,
				points_colors[i].b, 255);
		}
		else
			SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
		renderCircle(points[i], radius, 0.0);
	}
	// for circles
	SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
	float thickness = 0.0025*screenSizeInDistanceUnits;
	for (int i = 0; i < circles.size(); i++)
		renderCircle(circles[i].center, circles[i].radius, circles[i].radius-thickness);
	// bring it to the screen
	SDL_RenderPresent(renderer);
	return false;
}

Window::Pixel::Pixel() : i(0), j(0) {}
		
Window::Pixel::Pixel(int i, int j) : i(i), j(j) {}

Window::Pixel Window::pointToPixel(const Geometry2D::Vec2& point)
{
	Pixel p;
	p.i = point.x*pixelsPerDistanceUnit + screenSizeInPixels/2;
	p.j = -point.y*pixelsPerDistanceUnit + screenSizeInPixels/2;
	return p;
}

Window::Pixel Window::unitVectorToPixelDirection(const Geometry2D::Vec2& unit_vector)
{
	Pixel p;
	if (unit_vector.x > 0.0)
		p.i = 1;
	else
		p.i = -1;
	if (unit_vector.y > 0.0)
		p.j = -1;
	else
		p.j = 1;
	return p;
}

Window::BoundingBox Window::circleToBoundingBox(const Geometry2D::Vec2& center, float radius)
{
	BoundingBox bb;
	bb.lower_bounds = pointToPixel(Geometry2D::Vec2(center.x-radius, center.y+radius));
	bb.upper_bounds = pointToPixel(Geometry2D::Vec2(center.x+radius, center.y-radius));
	return bb;
}

Geometry2D::Vec2 Window::pixelToPoint(const Pixel& p)
{
	return Geometry2D::Vec2((p.i - screenSizeInPixels/2 + 0.5)/pixelsPerDistanceUnit,
		-(p.j - screenSizeInPixels/2 + 0.5)/pixelsPerDistanceUnit);
}

void Window::renderCircle(const Geometry2D::Vec2& center, float r_outer, float r_inner)
{
	// for convenience
	SDL_Renderer* renderer = allSdlPointers[sdlWindowCreationNumber].renderer;

	BoundingBox bb = circleToBoundingBox(center, r_outer);
	Geometry2D::Vec2 point;
	float distance;
	for (int i = bb.lower_bounds.i; i <= bb.upper_bounds.i; i++)
	{
		for (int j = bb.lower_bounds.j; j <= bb.upper_bounds.j; j++)
		{
			point = pixelToPoint(Pixel(i, j));
			distance = (point - center).norm();
			if (distance <= r_outer && distance >= r_inner)
				SDL_RenderDrawPoint(renderer, i, j);
			//else
			//	std::cout << "x=" << point.x << ", y=" << point.y << std::endl;
		}
	}
}

void Window::renderHalfPlanes(const std::vector<Geometry2D::HalfPlane2>& half_planes)
{
	// for convenience
	SDL_Renderer* renderer = allSdlPointers[sdlWindowCreationNumber].renderer;
	// render the complementary half-planes, in grey
	SDL_SetRenderDrawColor(renderer, 100, 100, 100, 255);
	Geometry2D::Vec2 point;
	for (int i = 0; i < screenSizeInPixels; i++)
	{
		for (int j = 0; j < screenSizeInPixels; j++)
		{
			point = pixelToPoint(Pixel(i, j));
			for (int k = 0; k < half_planes.size(); k++)
			{
				if (!half_planes[k].contains(point))
				{
					SDL_RenderDrawPoint(renderer, i, j);
					break;
				}
			}
		}	
	}
	// render boundary lines shifted outwards (into the "infeasible" space), in red
	SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
	for (int i = 0; i < half_planes.size(); i++)
		renderBoundaryLine(half_planes[i], 2);
	// render boundary lines, in white
	SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
	for (int i = 0; i < half_planes.size(); i++)
		renderBoundaryLine(half_planes[i]);
}

void Window::renderBoundaryLine(const Geometry2D::HalfPlane2& half_plane, int pixel_shift)
{
	SDL_Renderer* renderer = allSdlPointers[sdlWindowCreationNumber].renderer; //for convenience
	Geometry2D::Vec2 e_line(-half_plane.normal.y, half_plane.normal.x);
	Geometry2D::Vec2 anchor = half_plane.normal*half_plane.offset;
	Pixel p1(pointToPixel(anchor + e_line*2*screenSizeInDistanceUnits));
	Pixel p2(pointToPixel(anchor - e_line*2*screenSizeInDistanceUnits));
	Pixel shift_vector = unitVectorToPixelDirection(half_plane.normal);
	SDL_RenderDrawLine(renderer, p1.i + shift_vector.i*pixel_shift, p1.j + shift_vector.j*pixel_shift,
		p2.i + shift_vector.i*pixel_shift, p2.j + shift_vector.j*pixel_shift);
}
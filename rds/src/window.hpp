#ifndef  WINDOW_HPP
#define  WINDOW_HPP

#include "geometry.hpp"
#include <SDL2/SDL.h>
#include <vector>

// this class allows multiple windows,
// but not updating them in different threads.
class Window
{
public:
	Window(const char* name, float screen_size_in_distance_units,
		int screen_size_in_pixels, float frame_rate_in_Hz, bool* is_good);
	~Window();

	struct sdlColor
	{
		Uint8 r,g,b;
	};

	// returns the delay to the next frame in minutes,
	// after waiting for this very same delay
	float delayUntilNextFrameInMinutes();

	// returns true if the window has been closed, false otherwise
	bool render(const std::vector<Geometry2D::HalfPlane2>* half_planes,
		const std::vector<Geometry2D::Vec2>* points,
		const std::vector<sdlColor>& points_colors,
		const std::vector<AdditionalPrimitives2D::Circle>* circles,
		const std::vector<AdditionalPrimitives2D::Arrow>* arrows,
		const std::vector<Window::sdlColor>& arrows_colors);

	const float screenSizeInDistanceUnits;
	const int screenSizeInPixels;
	const float pixelsPerDistanceUnit;
	const float frameRateInHz;

private:
	// call it once before rendering to a new frame,
	// returns true if the sdl window has been closed
	bool sdlCheck();

	class Pixel
	{
	public:
		Pixel();
		Pixel(int i, int j);
		int i, j;
	};
	Pixel pointToPixel(const Geometry2D::Vec2& point) const;
	Pixel unitVectorToPixelDirection(const Geometry2D::Vec2& unit_vector);
	struct BoundingBox
	{
		Pixel lower_bounds;
		Pixel upper_bounds;
	};
	BoundingBox circleToBoundingBox(const Geometry2D::Vec2& center, float radius);
	Geometry2D::Vec2 pixelToPoint(const Pixel& p) const;

	void renderCircle(const Geometry2D::Vec2& center, float r_outer, float r_inner);
	void renderHalfPlanes(const std::vector<Geometry2D::HalfPlane2>& half_planes);
	void renderBoundaryLine(const Geometry2D::HalfPlane2& half_plane, int pixel_shift = 0);
	void renderArrows(const std::vector<AdditionalPrimitives2D::Arrow>& arrows,
		const std::vector<Window::sdlColor>& arrows_colors);

	static void killSdlWindow(int window_creation_number);
	struct SDLPointers
	{
		SDL_Window* window;
		SDL_Renderer* renderer;
	};
	static std::vector<SDLPointers> allSdlPointers;
	static bool sdlIsInitialized;

	const int sdlWindowCreationNumber;

	class HalfPlaneRenderer
	{
	public:
		HalfPlaneRenderer(const Window& win,
			const std::vector<Geometry2D::HalfPlane2>& halfplanes);
		~HalfPlaneRenderer();
		void render(SDL_Renderer* renderer);
	private:
		Geometry2D::Vec2 lower_left_corner_bounding_box;
		Geometry2D::Vec2 upper_left_corner_bounding_box;
		Geometry2D::Vec2 lower_right_corner_bounding_box;
		Geometry2D::Vec2 upper_right_corner_bounding_box;
		std::vector<Geometry2D::Vec2> feasible_points;
		SDL_Point* infeasible_points;
		int count_infeasible;
	};
};

#endif
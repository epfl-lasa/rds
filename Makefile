all: geometry gui demo_geo # list only what is ready to build

FLAGS = -std=c++11 -g -Wall


# DEMOS

demo: build/demo # target just for convenience

build/demo: src/demo.cpp build/libgeometry.a build/libgui.a
	g++ $(FLAGS) src/demo.cpp -Lbuild -lgeometry -lgui -lSDL2 -o build/demo

demo_geo: build/demo_geo # target just for convenience

build/demo_geo: src/demo_geo.cpp build/libgeometry.a build/libgui.a
	g++ $(FLAGS) src/demo_geo.cpp -Lbuild -lgeometry -lgui -lSDL2 -o build/demo_geo


# TESTS

TEST_LIB = -lgtest -lgtest_main -lpthread

test_geometry: build/test_geometry # target just for convenience

build/test_geometry: test/geometry_test.cpp build/libgeometry.a
	g++ $(FLAGS) test/geometry_test.cpp -Lbuild -lgeometry $(TEST_LIB) -o build/test_geometry


# ROS NODES

gui_client_node: build/gui_client_node # target just for convenience

build/gui_client_node: src/gui_client.cpp build/libgeometry.a build/libgui.a
	g++ $(FLAGS) src/gui_client.cpp -Lbuild -lgeometry -lgui -lSDL2 -o build/gui_client_node

gui_server_node: build/gui_server_node # target just for convenience

build/gui_server_node: build/.hi src/gui_server.cpp src/geometry.hpp
	g++ $(FLAGS) src/gui_server.cpp -o build/gui_server_node


# LIBRARIES

gui: build/libgui.a # target just for convenience

build/libgui.a: build/.hi src/geometry.hpp src/gui.cpp src/gui.hpp src/window.cpp src/window.hpp
	g++ $(FLAGS) -c src/gui.cpp -o build/gui.o 
	g++ $(FLAGS) -c src/window.cpp -o build/window.o
	ar rvs build/libgui.a build/gui.o build/window.o

geometry: build/libgeometry.a # target just for convenience

build/libgeometry.a: build/.hi src/geometry.hpp src/distance_minimizer.cpp src/distance_minimizer.hpp
	g++ $(FLAGS) -c src/distance_minimizer.cpp -o build/distance_minimizer.o
	ar rvs build/libgeometry.a build/distance_minimizer.o

# HOUSE KEEPING

clean:
	rm -rf build

build/.hi: # targets that depend on this will create the build folder (unless existing) before executing their rule
	if ! [ -d build ]; then \
		mkdir build; \
	fi
	touch .hi
	mv .hi build/.hi
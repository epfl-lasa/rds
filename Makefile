all: geometry gui demo

FLAGS = -std=c++11 -g -Wall

demo: geometry gui
	g++ $(FLAGS) src/demo.cpp -Lbuild -lgeometry -lgui -lSDL2 -o build/demo

gui_client: gui
	g++ $(FLAGS) src/gui_client.cpp -Lbuild -lgeometry -lgui -lSDL2 -o build/gui_client_node

gui_server: build
	g++ $(FLAGS) src/gui_server.cpp -o build/gui_server_node

gui: build
	g++ $(FLAGS) -c src/gui.cpp -o build/gui.o 
	g++ $(FLAGS) -c src/window.cpp -o build/window.o
	ar rvs build/libgui.a build/gui.o build/window.o

geometry: build
	g++ $(FLAGS) -c src/geometry.cpp -o build/geometry.o
	ar rvs build/libgeometry.a build/geometry.o

clean: build
	rm -rf build

build:
	mkdir build
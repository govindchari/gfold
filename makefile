default:
	g++ -std=c++14 -Iinclude -Isrc -Ilib/eigen -o build/main.o -c src/main.cpp
	g++ -std=c++14 -Iinclude -Isrc -Ilib/eigen -o build/AttitudeController.o -c src/gnc/AttitudeController.cpp
	g++ -o build/main.out build/main.o build/AttitudeController.o
	build/main.out
default:
	g++ -std=c++14 -Iinclude -Isrc -Ilib/eigen -o build/main.o -c src/main.cpp
	g++ -std=c++14 -Iinclude -Isrc -Ilib/eigen -o build/AttitudeController.o -c src/gnc/AttitudeController.cpp
	g++ -std=c++14 -Iinclude -Isrc -Ilib/eigen -o build/PositionController.o -c src/gnc/PositionController.cpp
	g++ -std=c++14 -Iinclude -Isrc -Ilib/eigen -o build/ThrustAllocator.o -c src/gnc/ThrustAllocator.cpp
	g++ -std=c++14 -Iinclude -Isrc -Ilib/eigen -o build/GenerateForcesMoments.o -c src/nsim/GenerateForcesMoments.cpp
	g++ -std=c++14 -Iinclude -Isrc -Ilib/eigen -o build/SaveData.o -c src/nsim/SaveData.cpp
	g++ -std=c++14 -Iinclude -Isrc -Ilib/eigen -o build/run6DOF.o -c src/nsim/simulations/run6DOF.cpp
	g++ -o build/main.out build/main.o build/AttitudeController.o build/PositionController.o build/ThrustAllocator.o build/GenerateForcesMoments.o build/SaveData.o build/run6DOF.o
	build/main.out
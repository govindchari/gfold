default:
	g++ -std=c++14 -I. -Ilib/eigen -o main.o -c src/main.cpp
	g++ -o main.out main.o
	./main.out
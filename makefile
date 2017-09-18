SRCS=$(wildcard *.cpp)
BINS=$(SRCS:.cpp=)



CXX = g++
CXXFLAGS = $(shell pkg-config --cflags ibex) 
LIBS	 = $(shell pkg-config --libs  ibex)


ifeq ($(DEBUG), yes)
CXXFLAGS := $(CXXFLAGS) -O0 -g -pg -Wall
else
CXXFLAGS := $(CXXFLAGS) -std=c++11
endif

collision-avoidance: collision-avoidance.o vibes.o sepProj.o interval_tools.o functions.o
	$(CXX) $^ -o collision-avoidance $(CXXFLAGS) $(LIBS)


vibes.o: vibes.h vibes.cpp
	$(CXX) vibes.cpp -c $(CXXFLAGS) $(LIBS)

sepProj.o: sepProj.h sepProj.cpp
	$(CXX) sepProj.cpp -c $(CXXFLAGS) $(LIBS)

interval_tools.o: interval_tools.h vibes.h sepProj.h interval_tools.cpp
	$(CXX) interval_tools.cpp -c $(CXXFLAGS) $(LIBS)

functions.o: functions.h interval_tools.h vibes.h functions.cpp
	$(CXX) functions.cpp -c $(CXXFLAGS) $(LIBS)

collision-avoidance.o:  vibes.h sepProj.h interval_tools.h functions.h json.hpp collision-avoidance.cpp
	$(CXX) collision-avoidance.cpp -c $(CXXFLAGS) $(LIBS)







clean:
	rm -rf *.bak rm -rf *.o collision-avoidance
	

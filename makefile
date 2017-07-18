SRCS=$(wildcard *.cpp)
BINS=$(SRCS:.cpp=)



CXX = g++
CXXFLAGS = $(shell pkg-config --cflags ibex) 
LIBS	 = $(shell pkg-config --libs  ibex)


ifeq ($(DEBUG), yes)
CXXFLAGS := $(CXXFLAGS) -O0 -g -pg -Wall
else
CXXFLAGS := $(CXXFLAGS)
endif

collision-avoidance: collision-avoidance.o vibes.o functions.o interval_tools.o
	$(CXX) $^ -o collision-avoidance $(CXXFLAGS) $(LIBS)


vibes.o: vibes.h

interval_tools.o: interval_tools.h

functions.o: functions.h

%.o: %.c
	$(CXX) -c $< -o $@ $(CXXFLAGS) $(LIBS)

clean:
	rm -rf *.bak rm -rf *.o collision-avoidance
	

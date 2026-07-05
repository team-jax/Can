CXX      = g++
CXXFLAGS = -Wall -Wextra -O2 -std=c++11
LDFLAGS  = -lpthread -lm

TARGET  = ak45_ctrl
SRCS    = ak45_36_socketcan_control.cpp main.cpp
OBJS    = $(SRCS:.cpp=.o)

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)

%.o: %.cpp ak45_36_socketcan_control.h
	$(CXX) $(CXXFLAGS) -c -o $@ $<

clean:
	rm -f $(OBJS) $(TARGET)

.PHONY: all clean

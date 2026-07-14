CXX      = g++
CXXFLAGS = -Wall -Wextra -O2 -std=c++11
LDFLAGS  = -lpthread -lm

TARGET      = ak45_ctrl
DEMO_TARGET = ak45_ctrl_demo
CORE_OBJ    = ak45_36_socketcan_control.o
OBJS        = $(CORE_OBJ) main.o demo_ramp.o

all: $(TARGET) $(DEMO_TARGET)

$(TARGET): $(CORE_OBJ) main.o
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)

$(DEMO_TARGET): $(CORE_OBJ) demo_ramp.o
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)

%.o: %.cpp ak45_36_socketcan_control.h
	$(CXX) $(CXXFLAGS) -c -o $@ $<

clean:
	rm -f $(OBJS) $(TARGET) $(DEMO_TARGET)

.PHONY: all clean

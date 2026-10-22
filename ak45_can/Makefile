# ak45_can — CAN 제어 라이브러리 + CLI. ROS 의존 없음.
# ROS2 노드(../ak45_ros2)는 여기 src/·include/ 를 직접 컴파일한다 — 복사본은 없다.

CXX      = g++
CXXFLAGS = -Wall -Wextra -O2 -std=c++11 -Iinclude
LDFLAGS  = -lpthread -lm

TARGET      = ak45_ctrl
DEMO_TARGET = ak45_ctrl_demo
CORE_OBJ    = ak45_36_socketcan_control.o
OBJS        = $(CORE_OBJ) main.o demo_ramp.o
HEADER      = include/ak45_36_socketcan_control.h

all: $(TARGET) $(DEMO_TARGET)

$(TARGET): $(CORE_OBJ) main.o
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)

$(DEMO_TARGET): $(CORE_OBJ) demo_ramp.o
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)

%.o: src/%.cpp $(HEADER)
	$(CXX) $(CXXFLAGS) -c -o $@ $<

%.o: apps/%.cpp $(HEADER)
	$(CXX) $(CXXFLAGS) -c -o $@ $<

clean:
	rm -f $(OBJS) $(TARGET) $(DEMO_TARGET)

.PHONY: all clean

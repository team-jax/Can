cmake_minimum_required(VERSION 3.8)
project(ak45_ros2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(Threads REQUIRED)

add_executable(ak45_node
  src/ak45_node.cpp
  src/ak45_36_socketcan_control.cpp
)

# -Wall -Wextra: 원본 Makefile이 이미 쓰는 옵션이라 원본도 통과가 보증된다.
# -Wpedantic: 원본 Makefile에 없어 보증되지 않으므로 우리 노드 파일에만 적용한다.
target_compile_options(ak45_node PRIVATE -Wall -Wextra)
set_source_files_properties(src/ak45_node.cpp
  PROPERTIES COMPILE_OPTIONS "-Wpedantic")

target_include_directories(ak45_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/ak45_ros2>
)

ament_target_dependencies(ak45_node
  rclcpp
  sensor_msgs
  diagnostic_msgs
)

# 라이브러리가 수신 pthread를 쓴다. -lm은 불필요 (§1.3)
target_link_libraries(ak45_node Threads::Threads)

install(TARGETS ak45_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

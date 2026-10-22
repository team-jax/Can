cmake_minimum_required(VERSION 3.8)
project(ak45_ros2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# CAN 계층은 이 패키지 밖(../ak45_can)에 있다. 소스 복사본을 두지 않고 원본을 직접
# 컴파일한다 — 예전처럼 src/ 에 사본을 두면 한쪽만 고쳐져 조용히 갈라진다.
# 경로를 바꿀 수 있게 캐시 변수로 뺀다.
set(AK45_CAN_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../ak45_can"
    CACHE PATH "ak45_can (CAN 제어 라이브러리) 디렉터리")
get_filename_component(AK45_CAN_DIR "${AK45_CAN_DIR}" ABSOLUTE)

if(NOT EXISTS "${AK45_CAN_DIR}/src/ak45_36_socketcan_control.cpp")
  message(FATAL_ERROR
    "ak45_can 을 찾을 수 없다: ${AK45_CAN_DIR}\n"
    "ros2_ws/src 에는 ak45_ros2 가 아니라 저장소 루트를 심링크해야 한다:\n"
    "  ln -sfn ~/바탕화면/Can ~/ros2_ws/src/Can\n"
    "또는 -DAK45_CAN_DIR=<경로> 로 직접 지정한다.")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(Threads REQUIRED)

add_executable(ak45_node
  src/ak45_node.cpp
  ${AK45_CAN_DIR}/src/ak45_36_socketcan_control.cpp
)

# -Wall -Wextra: ak45_can 의 Makefile 이 이미 쓰는 옵션이라 원본도 통과가 보증된다.
# -Wpedantic: 그 Makefile 에 없어 보증되지 않으므로 우리 노드 파일에만 적용한다.
target_compile_options(ak45_node PRIVATE -Wall -Wextra)
set_source_files_properties(src/ak45_node.cpp
  PROPERTIES COMPILE_OPTIONS "-Wpedantic")

target_include_directories(ak45_node PRIVATE ${AK45_CAN_DIR}/include)

ament_target_dependencies(ak45_node
  rclcpp
  sensor_msgs
  diagnostic_msgs
)

# 라이브러리가 수신 pthread를 쓴다. -lm은 불필요 (docs/AGENTS 대조 §1.3)
target_link_libraries(ak45_node Threads::Threads)

install(TARGETS ak45_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# 각도(도) 입력용 헬퍼. ros2 run ak45_ros2 ak45_deg 10
install(PROGRAMS scripts/ak45_deg
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

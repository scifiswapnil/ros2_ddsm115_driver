cmake_minimum_required(VERSION 3.5)
project(ddsm115_driver)

# use C++17
set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

# Include headers
include_directories(
  include
)

# Build node executable with embedded Communicator implementation
add_executable(ddsm115_node
  src/ddsm115_node.cpp
  src/DDSM115Communicator.cpp
)
ament_target_dependencies(ddsm115_node
  rclcpp
  sensor_msgs
  std_msgs
  yaml-cpp
)

# Install the node executable
install(
  TARGETS ddsm115_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()

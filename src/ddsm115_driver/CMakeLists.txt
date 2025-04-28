cmake_minimum_required(VERSION 3.5)
project(ddsm115_driver)

# use C++17
set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)


include_directories(
  include
)

add_library(ddsm115_comm
  src/DDSM115Communicator.cpp
)
ament_target_dependencies(ddsm115_comm rclcpp)
target_link_libraries(ddsm115_comm yaml-cpp)


add_executable(ddsm115_node
  src/ddsm115_node.cpp
)
target_link_libraries(ddsm115_node
  ddsm115_comm
)
ament_target_dependencies(ddsm115_node
  rclcpp sensor_msgs std_msgs yaml-cpp
)

install(
  DIRECTORY include/
  DESTINATION include/
)

install(TARGETS
  ddsm115_comm
  ddsm115_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()

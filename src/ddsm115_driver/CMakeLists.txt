cmake_minimum_required(VERSION 3.5)
project(ddsm115_driver)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(yaml-cpp REQUIRED)    # for parsing your motors.yaml
# find_package(ddsm115_driver REQUIRED) # if you have other libs (omit if not)

include_directories(
  include
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetMotorId.srv"
  "srv/QueryMotorId.srv"
  "srv/ChangeMotorMode.srv"
  DEPENDENCIES std_msgs
)

add_executable(ddsm115_ros2_node src/ddsm115_node.cpp src/DDSM115Communicator.cpp )

ament_target_dependencies(ddsm115_ros2_node
  rclcpp
  std_msgs
  sensor_msgs
)

target_link_libraries(ddsm115_ros2_node
  yaml-cpp
)

rosidl_target_interfaces(ddsm115_ros2_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

install(TARGETS ddsm115_ros2_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()

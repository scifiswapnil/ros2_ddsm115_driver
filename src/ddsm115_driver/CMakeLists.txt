cmake_minimum_required(VERSION 3.5)
project(ddsm115_driver)

# Enable C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)

# -----------------------------------------------------------------------------
# 1) Communicator library
# -----------------------------------------------------------------------------
add_library(ddsm115_driver_lib SHARED
  src/DDSM115Communicator.cpp
)
target_include_directories(ddsm115_driver_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(ddsm115_driver_lib
  rclcpp
)
target_link_libraries(ddsm115_driver_lib
  yaml-cpp
)
install(
  TARGETS ddsm115_driver_lib
  EXPORT export_ddsm115_driver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

# -----------------------------------------------------------------------------
# 2) Hardware interface plugin
# -----------------------------------------------------------------------------
add_library(ddsm115_hardware_interface SHARED
  src/ddsm115_hardware_interface.cpp
)
target_include_directories(ddsm115_hardware_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(ddsm115_hardware_interface
  rclcpp
  hardware_interface
  controller_interface
  pluginlib
)
target_link_libraries(ddsm115_hardware_interface
  ddsm115_driver_lib
)
pluginlib_export_plugin_description_file(
  pluginlib
  plugin_description.xml
)
install(
  TARGETS ddsm115_hardware_interface
  EXPORT export_ddsm115_driver
  LIBRARY DESTINATION lib
)

# -----------------------------------------------------------------------------
# 3) ROS2 node executable
# -----------------------------------------------------------------------------
add_executable(ddsm115_ros2_node
  src/ddsm115_node.cpp
)
target_include_directories(ddsm115_ros2_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(ddsm115_ros2_node
  rclcpp
  std_msgs
  sensor_msgs
)
target_link_libraries(ddsm115_ros2_node
  ddsm115_driver_lib
  yaml-cpp
)
install(
  TARGETS ddsm115_ros2_node
  EXPORT export_ddsm115_driver
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# -----------------------------------------------------------------------------
# 4) ROSIDL interface generation
# -----------------------------------------------------------------------------
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetMotorId.srv"
  "srv/QueryMotorId.srv"
  "srv/ChangeMotorMode.srv"
  DEPENDENCIES std_msgs
)
rosidl_target_interfaces(
  ddsm115_ros2_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

# -----------------------------------------------------------------------------
# Install headers, configs, and exports
# -----------------------------------------------------------------------------
install(
  DIRECTORY include/
  DESTINATION include/
)
install(
  FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)
install(
  EXPORT export_ddsm115_driver
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION share/${PROJECT_NAME}/cmake
)

install(
  PROGRAMS
    scripts/single_swerve_module_controller.py
    scripts/swerve_drive_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

# -----------------------------------------------------------------------------
# Export dependencies and targets
# -----------------------------------------------------------------------------
ament_export_dependencies(
  ament_cmake
  rclcpp
  std_msgs
  sensor_msgs
  rosidl_default_runtime
  hardware_interface
  controller_interface
  pluginlib
  yaml-cpp
)
ament_export_targets(export_ddsm115_driver HAS_LIBRARY_TARGET)

pluginlib_export_plugin_description_file(
  hardware_interface
  plugin_description.xml
)

install(
  FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
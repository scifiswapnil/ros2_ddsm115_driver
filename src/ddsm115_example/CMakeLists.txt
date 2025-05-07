cmake_minimum_required(VERSION 3.8)
project(ddsm115_example)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(controller_manager REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(ddsm115_driver REQUIRED)
find_package(xacro REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(rviz2 REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install the URDFs so they end up in share/ddsm115_example/urdf
install(
  DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

# Install controller configs
install(
  DIRECTORY config 
  DESTINATION share/${PROJECT_NAME}
)


ament_package()

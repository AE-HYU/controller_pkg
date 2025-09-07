cmake_minimum_required(VERSION 3.8)
project(controller_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(planning_custom_msgs REQUIRED)

# include directories
include_directories(include)

# create executables
add_executable(controller_node
  src/controller.cpp
  src/utils.cpp
)

# specify dependencies
ament_target_dependencies(controller_node
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
  ackermann_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  planning_custom_msgs
)

# install targets
install(TARGETS
  controller_node
  DESTINATION lib/${PROJECT_NAME}
)

# install directories
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
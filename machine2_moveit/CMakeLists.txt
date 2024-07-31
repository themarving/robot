cmake_minimum_required(VERSION 3.8)
project(machine2_moveit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# self added packages
find_package(pluginlib REQUIRED)
find_package(rcpputils REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(rclcpp REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(controller_interface REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(machine2_description REQUIRED)

find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)

# adding cpp executable
add_executable(move_group_interface src/move_group_interface.cpp)
target_include_directories(move_group_interface
  PUBLIC 
  include
)
ament_target_dependencies(move_group_interface
  machine2_description
  moveit_ros_planning
  moveit_core
  moveit_ros_planning_interface
  Eigen3
  pluginlib
  rcpputils
  hardware_interface
  kdl_parser
  rclcpp
  trajectory_msgs
  controller_interface
  realtime_tools
  Boost
)

# installing 
install(TARGETS 
  move_group_interface
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
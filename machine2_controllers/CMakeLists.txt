cmake_minimum_required(VERSION 3.8)
project(machine2_controllers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(control_msgs REQUIRED)
find_package(controller_interface REQUIRED)
find_package(trajectory_msgs REQUIRED)

# make controller plugin available in ros2_control
add_library(
  ${PROJECT_NAME} 
  SHARED
  src/machine2_controller.cpp
)
target_include_directories(${PROJECT_NAME} PRIVATE
  include
  "${SERIAL_INCLUDE_DIRS}"
)
target_link_libraries(${PROJECT_NAME}
  "${SERIAL_LDFLAGS}"
)
ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  rcl_interfaces
  realtime_tools
  control_msgs
  controller_interface
  trajectory_msgs
)

# INSTALLING PLUGIN THROUGH XML FILE
pluginlib_export_plugin_description_file(controller_interface src/machine2_controller_plugin.xml)

# INSTALL
install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

install(
  DIRECTORY include 
  DESTINATION include
)

ament_export_include_directories(
  include
)

ament_export_libraries(
  ${PROJECT_NAME}
)

ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  rcl_interfaces
  realtime_tools
  control_msgs
  controller_interface
  trajectory_msgs
)

ament_package()
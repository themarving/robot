cmake_minimum_required(VERSION 3.8)
project(machine2_hardware_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# self added packages
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(custom_interfaces REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

# make hardware plugin available in ros2_control
add_library(
  ${PROJECT_NAME} 
  SHARED
  src/machine2_hardware_interface.cpp
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
  custom_interfaces
  std_msgs
  rcl_interfaces
)

# INSTALLING PLUGIN THROUGH XML FILE
pluginlib_export_plugin_description_file(hardware_interface src/machine2_hardware_interface_plugin.xml)

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
  custom_interfaces
  std_msgs
  rcl_interfaces
)

ament_package()
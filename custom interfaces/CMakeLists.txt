cmake_minimum_required(VERSION 3.8)
project(custom_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# SELF ADDED
find_package(rosidl_default_generators REQUIRED)

# creating source code from message
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/UltrasonicSensorStatus.msg"
  "msg/FaceCoordinates.msg"
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()

cmake_minimum_required(VERSION 3.8)
project(shared_lib)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rosidl_default_generators REQUIRED)

add_library(shared_lib INTERFACE)

target_include_directories(shared_lib INTERFACE 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)


ament_target_dependencies(
  shared_lib INTERFACE
  rclcpp
  std_msgs
  sensor_msgs
  std_srvs
  diagnostic_msgs
  rclcpp_action
  rosidl_default_generators
)

install(
    DIRECTORY include/
    DESTINATION include)

ament_export_dependencies(
  rclcpp
  std_msgs
  sensor_msgs
  std_srvs
  diagnostic_msgs
  rclcpp_action
  rosidl_default_generators
)

ament_package()
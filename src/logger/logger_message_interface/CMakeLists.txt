cmake_minimum_required(VERSION 3.8)
project(logger_message_interface)

find_package(logger_message_generator REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(log_class REQUIRED)

# Confirm the generator has already produced the file
if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/msg/Log.msg")
  message(FATAL_ERROR "Log.msg has not been generated. Build the generator project first.")
endif()

# ROSIDL interface generation
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Log.msg"
)
ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(empty_executable src/main.cpp src/size_test.cpp)

ament_target_dependencies(empty_executable 
  log_class
)

install(TARGETS empty_executable
  DESTINATION lib/${PROJECT_NAME})

ament_package()
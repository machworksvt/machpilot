cmake_minimum_required(VERSION 3.8)
project(logger_message)

find_package(logger_message_generator REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Confirm the generator has already produced the file
if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/msg/LoggerMessage.msg")
  message(FATAL_ERROR "LoggerMessage.msg has not been generated! Build the generator project first.")
endif()

# ROSIDL interface generation
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LoggerMessage.msg"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(empty_executable src/main.cpp src/size_test.cpp)

install(TARGETS empty_executable
  DESTINATION lib/${PROJECT_NAME})

ament_package()
cmake_minimum_required(VERSION 3.8)

set(CMAKE_CXX_COMPILER "g++")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)


project(logger)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(rclcpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

include_directories(
  "src"
)

execute_process(
  COMMAND cmake -S ${CMAKE_SOURCE_DIR}/message_preprossessor -B ${CMAKE_CURRENT_BINARY_DIR}/preprossessor_build
)

#rosidl_generate_interfaces(${PROJECT_NAME}
#  "${CMAKE_CURRENT_BINARY_DIR}:LoggerMessage.msg"
#)

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


add_executable(logger_node
  src/main.cpp
)

ament_target_dependencies(logger_node
    rclcpp 
)

install(TARGETS
  logger_node
  DESTINATION lib/${PROJECT_NAME}
)

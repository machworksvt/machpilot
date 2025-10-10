# ------------------------------------------------------------------------------------------
#                    THIS FILE IS A TEMPLATE, REMOVE THIS WHEN IN USE
# ------------------------------------------------------------------------------------------
# 1. find-replace all <name>, <node_name>, and <driver_name> with the real names
#
# 2. add more find_package() lines as needed
# 
# 3. check if the commented-out lines are in use, especially if custom interfaces are in use

cmake_minimum_required(VERSION 3.8)
project(state_estimator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17) # cpp standard, 17 for our project
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_LIBROBCONT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../dependencies/librobotcontrol/library/src")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)

# --- Library (optional) ---
# add_library(${PROJECT_NAME}_lib
#   src/your_library.cpp
# )
# target_include_directories(${PROJECT_NAME}_lib PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
# )
# ament_target_dependencies(${PROJECT_NAME}_lib
#   rclcpp
#   std_msgs
# )

# --- Node Executable ---
add_executable(state_estimator_node src/state_estimator_node.cpp
  src/state_estimator.c
  ${CMAKE_LIBROBCONT_DIR}/math/vector.c
  ${CMAKE_LIBROBCONT_DIR}/math/matrix.c
  ${CMAKE_LIBROBCONT_DIR}/math/kalman.c
  ${CMAKE_LIBROBCONT_DIR}/math/matrix.c
  ${CMAKE_LIBROBCONT_DIR}/math/other.c
  ${CMAKE_LIBROBCONT_DIR}/math/algebra_common.c
  ${CMAKE_LIBROBCONT_DIR}/math/algebra.c
  ${CMAKE_LIBROBCONT_DIR}/time.c)

target_include_directories(state_estimator_node PUBLIC 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/state_estimator>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../dependencies/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../dependencies/librobotcontrol/library/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(state_estimator_node 
  rclcpp 
  rclcpp_action
  rosidl_default_generators
  std_msgs
  sensor_msgs
  diagnostic_msgs)

target_link_libraries(state_estimator_node pthread) # good to include for system calls

# --- Installation ---
install(TARGETS state_estimator_node #${PROJECT_NAME}_lib # libraries
#  EXPORT export_${PROJECT_NAME} # for child dependencies
  DESTINATION lib/${PROJECT_NAME}
)

# the commented lines will be used when launch files are implemented, just uncomment
install(DIRECTORY
  #launch
  #config
  DESTINATION share/${PROJECT_NAME}
)

# --- Export interfaces for downstream packages (optional) ---
# ament_export_include_directories(include)
# ament_export_libraries(${PROJECT_NAME}_lib)
# ament_export_dependencies(rclcpp std_msgs sensor_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  # set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
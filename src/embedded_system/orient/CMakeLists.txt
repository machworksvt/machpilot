# ------------------------------------------------------------------------------------------
#                    THIS FILE IS A TEMPLATE, REMOVE THIS WHEN IN USE
# ------------------------------------------------------------------------------------------
# 1. find-replace all <name>, <node_name>, and <driver_name> with the real names
#
# 2. add more find_package() lines as needed
# 
# 3. check if the commented-out lines are in use, especially if custom interfaces are in use

cmake_minimum_required(VERSION 3.8)
project(orient)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17) # cpp standard, 17 for our project
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

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
add_executable(bno055_node src/bno055_node.cpp)

target_include_directories(bno055_node PUBLIC 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/orient>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../dependencies/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(bno055_node
  rclcpp 
  rclcpp_lifecycle 
  rclcpp_action
  rosidl_default_generators)

target_link_libraries(bno055_node pthread) # good to include for system calls

# --- Installation ---
install(TARGETS bno055_node #${PROJECT_NAME}_lib # libraries
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
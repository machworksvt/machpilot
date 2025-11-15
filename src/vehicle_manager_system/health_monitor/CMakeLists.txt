cmake_minimum_required(VERSION 3.8)
project(health_monitor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 18) # cpp standard, 18 for our project
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# --- Node Executable ---
add_executable(health_monitor src/errorMonitor.cpp)

target_include_directories(health_monitor PUBLIC 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/health_monitor>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../dependencies/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(health_monitor 
  rclcpp 
  rclcpp_lifecycle 
  rclcpp_action
  rosidl_default_generators)

# --- Installation ---
install(TARGETS health_monitor ${health_monitor}_lib # libraries
  EXPORT export_${health_monitor} # for child dependencies
  DESTINATION lib/${health_monitor}
)

# the commented lines will be used when launch files are implemented, just uncomment
install(DIRECTORY
  #launch
  #config
  DESTINATION share/${health_monitor}
)

# --- Export interfaces for downstream packages (optional) ---
ament_export_include_directories(include)
ament_export_libraries(${health_monitor}_lib)
ament_export_dependencies(rclcpp std_msgs sensor_msgs)

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
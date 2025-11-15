# ------------------------------------------------------------------------------------------
#                    THIS FILE IS A TEMPLATE, REMOVE THIS WHEN IN USE
# ------------------------------------------------------------------------------------------
# 1. find-replace all <name>, <node_name>, and <driver_name> with the real names
#
# 2. add more find_package() lines as needed
# 
# 3. check if the commented-out lines are in use, especially if custom interfaces are in use

cmake_minimum_required(VERSION 3.8)
project(ublox_gps_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17) # cpp standard, 17 for our project
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)

# --- Library (optional) ---
include_directories(
  include
  ../dependencies/include
)

# UART C lib
add_library(uart_lib STATIC
  ../dependencies/src/uart.c
)

# ublox_messages lib
add_library(ublox_messages STATIC  
  src/ublox_messages.cpp
)

# uart_interface lib
add_library(uart_interface STATIC  
  src/uart_interface.cpp
)

target_link_libraries(uart_interface
  uart_lib
)

# ublox_parser library
add_library(ublox_parser STATIC 
  src/ublox_parser.cpp 
)

target_link_libraries(ublox_parser
  ublox_messages
)

# GPS Node exe
add_executable(gps_node 
  src/gps_node.cpp 
  src/main.cpp 
)

target_link_libraries(gps_node
  uart_interface
  ublox_parser
  ublox_messages
)

ament_target_dependencies(gps_node
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  diagnostic_msgs
)

# --- Installation ---
install(TARGETS gps_node 
  DESTINATION lib/${PROJECT_NAME}
)

# the commented lines will be used when launch files are implemented, just uncomment
install(DIRECTORY include/
  #launch
  #config
  DESTINATION include
)

# --- Export interfaces for downstream packages (optional) ---
# ament_export_include_directories(include)
# ament_export_libraries(${PROJECT_NAME}_lib)
# ament_export_dependencies(rclcpp std_msgs sensor_msgs)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_checksum
    test/test_checksum.cpp
  )
  target_link_libraries(test_checksum
    ublox_messages
  )

  #[[
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  # set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  ]]
   set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  set(ament_cmake_lint_cmake_FOUND TRUE)
  set(ament_cmake_flake8_FOUND TRUE)
  set(ament_cmake_pep257_FOUND TRUE)
  set(ament_cmake_xmllint_FOUND TRUE)
endif()

ament_package()
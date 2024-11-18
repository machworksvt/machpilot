# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(std_srvs REQUIRED)

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${diagnostic_msgs_INCLUDE_DIRS}
  ${std_srvs_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}/../ext_deps/include
)

add_executable(
    # ADD THE NODE NAME WITH .cpp EXTENSION #
)

ament_target_dependencies(
  # ADD THE NODE NAME HERE #
  rclcpp
  sensor_msgs
  std_msgs
  diagnostic_msgs
  std_srvs
)

install(TARGETS # ADD THE NODE NAME HERE #
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

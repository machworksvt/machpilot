cmake_minimum_required(VERSION 3.10)

# Explicitly set the compilers to g++ and gcc
set(CMAKE_CXX_COMPILER "g++")

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Add the executable
project(logger)

# Find necessary ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Num.msg"
 )

# Add include directories
include_directories(
  "src"
)

# Add the executable for the ROS 2 node.
add_executable(logger
  "src/main.cpp"
)

# Link the executable against its dependencies
ament_target_dependencies(logger
  "rclcpp"
  "std_msgs"
)

# Install the executable
install(TARGETS
  logger
  DESTINATION lib/${PROJECT_NAME}
)

# Final ament package call
ament_package()
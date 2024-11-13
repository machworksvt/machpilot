cmake_minimum_required(VERSION 3.8)

# Define the project
project(ros2_rfd900x)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(MAVSDK REQUIRED)

# Add the path to the installed MAVSDK CMake files (adjust if necessary)
set(CMAKE_PREFIX_PATH "${CMAKE_PREFIX_PATH}:/usr/local/lib/cmake/MAVSDK")

# Explicitly add the MAVSDK include directory
include_directories(/usr/local/include/mavsdk)

# Add the path to the rclcpp library (to help linker find it)
link_directories("/opt/ros/jazzy/lib")  # Add the ROS 2 library directory explicitly

# Add the MAVSDK library directory (adjust if necessary)
link_directories("/usr/local/lib")  # Add MAVSDK library path explicitly

# Add your source files
add_executable(rfd900x_node src/rfd900x_node.cpp)
add_executable(rfd900x_receiver_node src/rfd900x_receiver_node.cpp)  # New executable for receiver node

# Link the executables to the necessary libraries
target_link_libraries(rfd900x_node rclcpp mavsdk)  # Link to rclcpp and mavsdk explicitly
target_link_libraries(rfd900x_receiver_node rclcpp mavsdk)  # Link to rclcpp and mavsdk explicitly

# Ensure ament dependencies are included, including MAVSDK and rclcpp
ament_target_dependencies(rfd900x_node rclcpp MAVSDK)
ament_target_dependencies(rfd900x_receiver_node rclcpp MAVSDK)  # Add dependencies for receiver node

# Install the executables
install(TARGETS rfd900x_node rfd900x_receiver_node
  DESTINATION lib/${PROJECT_NAME})

# Ament package hooks
ament_package()

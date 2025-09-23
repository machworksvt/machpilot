#command to build and run cmake --build \build;./build/FSMPilot

cmake_minimum_required(VERSION 3.10)

# Explicitly set the compilers to g++ and gcc
set(CMAKE_C_COMPILER "gcc")
set(CMAKE_CXX_COMPILER "g++")

project(FSMPilot)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Add the executable
cmake_minimum_required(VERSION 3.10)
project(fsm_ros_wrapper)

# Explicitly set the compilers
set(CMAKE_C_COMPILER "gcc")
set(CMAKE_CXX_COMPILER "g++")

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Find necessary ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Add include directories
include_directories(
  "src"
)

# Add the executable for the ROS 2 node.
# List all source files: the new ROS wrapper files AND your existing state machine implementation files.
add_executable(state_machine_node
    src/ros_main.cpp
    src/state_machine_node.cpp
    src/FSMPilot.cpp
    src/motorManager.cpp
    src/ScreenManager.cpp
    src/ErrorManager.cpp
    src/FireManager.cpp
    src/FireManager.cpp
    src/ErrorManagerList.cpp
    src/FSMPilotStates.cpp
)

# Link the executable against its dependencies
ament_target_dependencies(state_machine_node
  "rclcpp"
  "std_msgs"
)

# Install the executable
install(TARGETS
  state_machine_node
  DESTINATION lib/${PROJECT_NAME}
)

# Final ament package call
ament_package()
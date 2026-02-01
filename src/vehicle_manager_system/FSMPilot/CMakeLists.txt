cmake_minimum_required(VERSION 3.10)

# Explicitly set the compilers to g++ and gcc
set(CMAKE_C_COMPILER "gcc")
set(CMAKE_CXX_COMPILER "g++")

project(FSMPilot)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

include_directories("${CMAKE_SOURCE_DIR}/src")

# Add the executable
add_executable(FSMPilot src/main.cpp src/events.cpp src/FSMPilot.cpp src/subsystems.cpp)

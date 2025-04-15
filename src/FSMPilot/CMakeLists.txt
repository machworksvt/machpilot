cmake_minimum_required(VERSION 3.10)

# Explicitly set the compilers to g++ and gcc
set(CMAKE_C_COMPILER "gcc")
set(CMAKE_CXX_COMPILER "g++")

project(FSMPilot)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Add include directory for tinyfsm
include_directories("${CMAKE_SOURCE_DIR}/tinyfsm/include")

# Add the executable
add_executable(FSMPilot main.cpp FSMPilot.cpp motorManager.cpp ScreenManager.cpp ErrorManager.cpp FireManager.cpp FireManager.cpp ErrorManagerList.cpp)

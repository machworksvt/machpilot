cmake_minimum_required(VERSION 3.5)
project(interfaces)
set(CMAKE_VERBOSE_MAKEFILE ON)
# Set CMAKE_BUILD_TYPE
set(CMAKE_BUILD_TYPE Release)
# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(ALIAS_ROS2_TF2)
  add_definitions(-Dtf2=ros2_tf2)
  add_definitions(-Dtf2_ros=ros2_tf2_ros)
  add_definitions(-Dmessage_filters=ros2_message_filters)
endif()
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rosidl_typesupport_c REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package( class_loader REQUIRED)
find_package( console_bridge REQUIRED)
find_package( rclcpp REQUIRED)
find_package( rcutils REQUIRED)
find_package( std_msgs REQUIRED)
include_directories("C:/Program Files/MATLAB/R2024b/extern/include")
include_directories("C:/Program Files/MATLAB/R2024b/extern/include/MatlabDataArray")
include_directories("C:/Program Files/MATLAB/R2024b/toolbox/ros/include/ros2")
include_directories("include/interfaces")
set (CMAKE_SKIP_BUILD_RPATH false)
set (CMAKE_BUILD_WITH_INSTALL_RPATH true)
set (CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
set (CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(msg_files
  "msg/EngineData.msg"
  "msg/Errors.msg"
  "msg/FuelAmbient.msg"
  "msg/GlowPlugs.msg"
  "msg/LastRunInfo.msg"
  "msg/NgReg.msg"
  "msg/PumpRpm.msg"
  "msg/Statistics.msg"
  "msg/SystemInfo.msg"
  "msg/SystemInfo2.msg"
  "msg/VoltageCurrent.msg"
)
set(srv_files
)
set(action_files
)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES 
  std_msgs
)
link_directories("C:/Program Files/MATLAB/R2024b/extern/lib/win64/microsoft")
# Library
add_library(
    interfaces_matlab 
    SHARED
    src/interfaces_EngineData_message.cpp
    src/interfaces_Errors_message.cpp
    src/interfaces_FuelAmbient_message.cpp
    src/interfaces_GlowPlugs_message.cpp
    src/interfaces_LastRunInfo_message.cpp
    src/interfaces_NgReg_message.cpp
    src/interfaces_PumpRpm_message.cpp
    src/interfaces_Statistics_message.cpp
    src/interfaces_SystemInfo_message.cpp
    src/interfaces_SystemInfo2_message.cpp
    src/interfaces_VoltageCurrent_message.cpp
)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
target_compile_options(interfaces_matlab PUBLIC 
  $<$<OR:$<COMPILE_LANGUAGE:CXX>,$<COMPILE_LANGUAGE:C>>:
  -Wall -Wextra -Wpedantic
  >)
endif()
target_include_directories(interfaces_matlab PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries( interfaces_matlab
	"libMatlabDataArray"
	${CMAKE_DL_LIBS}
	)
ament_target_dependencies(
  interfaces_matlab
  "class_loader"
  "console_bridge"
  "rclcpp"
  "rcutils"
  "std_msgs"
)
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(interfaces_matlab "${cpp_typesupport_target}")
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(interfaces_matlab PRIVATE "INTERFACES_BUILDING_LIBRARY")
install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS interfaces_matlab
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install (DIRECTORY m/ DESTINATION m/ FILES_MATCHING PATTERN "*.m")
ament_export_include_directories(
  include
)
ament_export_libraries(
  interfaces_matlab
)
foreach(file_i ${CUDA_BINS})
add_custom_command(
                   TARGET 
                   POST_BUILD
                   COMMAND ${CMAKE_COMMAND}
                   ARGS -E copy ${PROJECT_SOURCE_DIR}/src/${file_i} ${CMAKE_INSTALL_PREFIX}/_ert_rtw/${file_i}
				   )
add_custom_command(
                   TARGET 
                   POST_BUILD
                   COMMAND ${CMAKE_COMMAND}
                   ARGS -E copy ${PROJECT_SOURCE_DIR}/src/${file_i} ${CMAKE_INSTALL_PREFIX}/codegen/exe//${file_i}
				   )				   
endforeach(file_i)
ament_package()
# Generated on Sun Feb 23 23:51:55 2025

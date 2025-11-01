cmake_minimum_required(VERSION 3.8)
project(orient2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(shared_lib REQUIRED)

# Glob together all .c files from the mpu9250 repo ALL SUBFOLDERS WITH .C FILES
file(GLOB MPU9250_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/src/mpu9250/src/*.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/mpu9250/interface/*.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/mpu9250/example/*.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/mpu9250/test/*.c"
  )

# Make a library out of it
add_library(mpu9250 STATIC ${MPU9250_SOURCES})

# include it
target_include_directories(mpu9250 PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/mpu9250/src>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/mpu9250/interface>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/mpu9250/example>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/mpu9250/test>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../ext_deps/include>
)

add_executable(mpu9250_node src/mpu9250_node.cpp)
target_include_directories(mpu9250_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/mpu9250/src>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/mpu9250/interface>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/mpu9250/example>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/mpu9250/test>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../ext_deps/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(mpu9250_node mpu9250)
target_compile_features(mpu9250_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(mpu9250_node shared_lib)

install(TARGETS mpu9250_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

# ------------------------------------------------------------------------------------------
#                    THIS FILE IS A TEMPLATE, REMOVE THIS WHEN IN USE
# ------------------------------------------------------------------------------------------
# 1. find-replace all <name>, <node_name>, and <driver_name> with the real names
#
# 2. add more find_package() lines as needed
# 
# 3. check if the commented-out lines are in use, especially if custom interfaces are in use

cmake_minimum_required(VERSION 3.8)
project(orient2)

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
add_executable(mpu9250_node src/mpu9250_node.cpp src/mpu9250_driver.c)

target_include_directories(mpu9250_node PUBLIC 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/orient2>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../dependencies/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(mpu9250_node 
  rclcpp 
  rclcpp_lifecycle 
  rclcpp_action
  rosidl_default_generators)

target_link_libraries(mpu9250_node pthread) # good to include for system calls

# --- Installation ---
install(TARGETS mpu9250_node #${PROJECT_NAME}_lib # libraries
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
cmake_minimum_required(VERSION 3.8)
project(orient2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

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
)

add_executable(mpu9250_node src/mpu9250_node.cpp)
target_include_directories(mpu9250_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/mpu9250/src>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/mpu9250/interface>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/mpu9250/example>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/mpu9250/test>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(mpu9250_node mpu9250)
target_compile_features(mpu9250_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  mpu9250_node
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
)

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

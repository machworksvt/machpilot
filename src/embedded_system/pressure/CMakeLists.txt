cmake_minimum_required(VERSION 3.8)
project(pressure)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# Glob together all .c files from the bmp390 repo ALL SUBFOLDERS WITH .C FILES
file(GLOB BMP390_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/src/bmp390/src/*.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/bmp390/interface/*.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/bmp390/example/*.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/bmp390/test/*.c"
)

# Make a library out of it
add_library(bmp390 STATIC ${BMP390_SOURCES})

# include it
target_include_directories(bmp390 PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/bmp390/src>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/bmp390/interface>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/bmp390/example>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/bmp390/test>
)

add_executable(bmp390_node src/bmp390_node2.cpp)
target_include_directories(bmp390_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../dependencies/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/bmp390/src>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/bmp390/interface>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/bmp390/example>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/bmp390/test>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(bmp390_node bmp390)
target_compile_features(bmp390_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  bmp390_node
  "rclcpp"
  "rclcpp_action"
  "rclcpp_lifecycle"
  "std_srvs"
  "std_msgs"
  "sensor_msgs"
)

install(TARGETS bmp390_node
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

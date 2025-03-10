cmake_minimum_required(VERSION 3.8)
project(orient)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(diagnostic_msgs REQUIRED)

file(GLOB BNO055_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/include/airspeed/*.cpp"
  )

add_library(bno055 STATIC ${BNO055_SOURCES})

target_include_directories(bno055 PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/airspeed>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../ext_deps/include>
  )

add_executable(bno055_node src/bno055_node.cpp)
target_include_directories(bno055_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/airspeed>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../ext_deps/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(bno055_node bno055)
target_compile_features(bno055_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  bno055_node
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
  "std_srvs"
  "diagnostic_msgs"
)

install(TARGETS bno055_node
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

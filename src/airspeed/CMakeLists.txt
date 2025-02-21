cmake_minimum_required(VERSION 3.8)
project(airspeed)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

file(GLOB MS4525DO_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/include/airspeed/*.cpp"
  )

add_library(ms4525do STATIC ${MS4525DO_SOURCES})

target_include_directories(ms4525do PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/airspeed>
  )

add_executable(ms4525do_node src/ms4525do_node.cpp)
target_include_directories(ms4525do_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/airspeed>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(ms4525do_node ms4525do)
target_compile_features(ms4525do_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  ms4525do_node
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
)

install(TARGETS ms4525do_node
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

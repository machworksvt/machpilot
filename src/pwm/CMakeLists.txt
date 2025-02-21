cmake_minimum_required(VERSION 3.8)
project(pwm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Glob together all .c files from the pca9685 repo ALL SUBFOLDERS WITH .C FILES
file(GLOB PCA9685_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/include/pwm/*.cpp"
  )

# Make a library out of it
add_library(pca9685 STATIC ${PCA9685_SOURCES})

# include it
target_include_directories(pca9685 PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/pwm>
)

add_executable(pca9685_node src/pca9685_node.cpp)
add_executable(pca9685_node_2 src/pca9685_node_2.cpp)

target_include_directories(pca9685_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/pwm>
  $<INSTALL_INTERFACE:include>)
target_include_directories(pca9685_node_2 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/pwm>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(pca9685_node pca9685)
target_link_libraries(pca9685_node_2 pca9685)
target_compile_features(pca9685_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  pca9685_node
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
)
target_compile_features(pca9685_node_2 PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  pca9685_node_2
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
)


install(TARGETS pca9685_node
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS pca9685_node_2
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


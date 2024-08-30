cmake_minimum_required(VERSION 3.8)
project(drivers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(FetchContent)

FetchContent_Declare(
    Adafruit_BNO055
    GIT_REPOSITORY https://github.com/adafruit/Adafruit_BNO055.git
    GIT_TAG        1.6.3
)

FetchContent_MakeAvailable(Adafruit_BNO055)

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(BNO055_manager src/BNO055_manager.cpp)
ament_target_dependencies(BNO055_manager rclcpp sensor_msgs)
target_link_libraries(BNO055_manager
  ${sensor_msgs_LIBRARIES}
)
target_include_directories(BNO055_manager PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${CMAKE_BINARY_DIR}/_deps/adafruit_bno055-src)
target_compile_features(BNO055_manager PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS BNO055_manager
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

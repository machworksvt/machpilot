cmake_minimum_required(VERSION 3.8)
project(communicator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(interfaces REQUIRED)
find_package(MAVSDK REQUIRED)

add_executable(datalink src/datalink.cpp)
add_executable(groundstation src/groundstation.cpp)

target_link_libraries(datalink MAVSDK::mavsdk)
target_link_libraries(groundstation MAVSDK::mavsdk)

target_include_directories(datalink PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${MAVSDK_INCLUDE_DIRS})
target_include_directories(groundstation PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${MAVSDK_INCLUDE_DIRS})
target_compile_features(datalink PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_features(groundstation PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  groundstation
  "rclcpp"
  "std_msgs"
  "interfaces"
)
ament_target_dependencies(
  datalink
  "rclcpp"
  "std_msgs"
  "interfaces"
)

install(TARGETS datalink
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS groundstation
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

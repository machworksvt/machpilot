cmake_minimum_required(VERSION 3.8)
project(log_file_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(logger_message_interface REQUIRED)
find_package(log_class REQUIRED)

add_executable(log_file_writer_node src/log_file_writer_node.cpp src/file_writing_assert.cpp)
add_executable(log_file_reader src/log_file_reader.cpp src/file_writing_assert.cpp)

target_include_directories(log_file_writer_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(log_file_writer_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  log_file_writer_node
  "rclcpp"
  "logger_message_interface"
  "log_class"
)

ament_target_dependencies(
  log_file_reader
  "log_class"
)

install(TARGETS log_file_writer_node
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS log_file_reader
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

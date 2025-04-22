cmake_minimum_required(VERSION 3.15)
project(gnc)

# 1) For IntelliSense:
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  #add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 2) ROS‑2 deps
find_package(ament_cmake REQUIRED)
find_package(rclcpp       REQUIRED)

# 3) Vendor CasADi in-tree
add_subdirectory(
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/casadi
  ${CMAKE_CURRENT_BINARY_DIR}/casadi_build
  EXCLUDE_FROM_ALL
)

# 4) Create alias if CasADi’s own target is called `casadi`
add_library(casadi::casadi ALIAS casadi)

# 5) Your test node
add_executable(casadi_test_node src/casadi_test_node.cpp)
ament_target_dependencies(casadi_test_node rclcpp)

# 6) Link to CasADi
target_link_libraries(casadi_test_node
  casadi::casadi
)

install(TARGETS casadi_test_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()

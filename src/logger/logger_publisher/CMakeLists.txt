cmake_minimum_required(VERSION 3.8)
project(logger_publisher)

set(CMAKE_CXX_STANDARD 17)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(logger_message_interface REQUIRED)  
find_package(log_class REQUIRED)

####################################
# CPP Shared Library Block [BEGIN] #
# vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv #
include_directories(include/)

add_library(${PROJECT_NAME} SHARED
    src/logger_publisher.cpp
)

target_include_directories(${PROJECT_NAME}
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

# Export the target so downstream packages can link against it
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_export_libraries(${PROJECT_NAME})

ament_export_dependencies(
    rclcpp
    logger_message_interface
    log_class
)

# Install headers
install(
    DIRECTORY include/
    DESTINATION include
)

# Install the library binary and export config
install(
    TARGETS ${PROJECT_NAME}
    EXPORT export_${PROJECT_NAME}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)
# ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ #
# CPP Shared Library Block [END] #
##################################

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif()

ament_target_dependencies(logger_publisher rclcpp logger_message_interface log_class)
ament_package()
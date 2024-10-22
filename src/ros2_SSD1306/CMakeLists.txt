cmake_minimum_required(VERSION 3.8)
project(ros2_SSD1306)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(${PROJECT_NAME} src/camera_sub.cpp src/SSD1306_OLED_graphics.cpp 
                       src/SSD1306_OLED_font.cpp src/SSD1306_OLED_Print.cpp src/SSD1306_OLED.cpp)
                       
ament_target_dependencies(${PROJECT_NAME} rclcpp std_msgs sensor_msgs)

# target_include_directories(display PRIVATE ${CMAKE_SOURCE_DIR}/external)
# target_link_libraries(display ${CMAKE_SOURCE_DIR}/external/bcm2835-1.55)
# add_library(bcm2835-1.55 SHARED IMPORTED GLOBAL)
# set_target_properties(bcm2835-1.55 PROPERTIES IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/external)
#target_link_libraries(display bcm2835-1.55)

target_link_directories(${PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/src/ros2_SSD1306/external/)
target_link_libraries(${PROJECT_NAME} bcm2835)

target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/include)

install(TARGETS
${PROJECT_NAME}
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

cmake_minimum_required(VERSION 3.8)
project(logger_message_generator)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(message_preprocessor message_preprocessor.cpp)

# Output goes to the ROS package msg folder
set(GENERATED_MSG "${CMAKE_CURRENT_LIST_DIR}/../logger_message/msg/LoggerMessage.msg")
set(GENERATED_TEST "${CMAKE_CURRENT_LIST_DIR}/../logger_message/src/size_test.cpp")

add_custom_target(run_generator ALL
  DEPENDS message_preprocessor
  COMMAND message_preprocessor "${GENERATED_MSG}" "${GENERATED_TEST}"
  COMMENT "Generating ${GENERATED_MSG} from C++ preprocessor"
)


install(TARGETS message_preprocessor
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

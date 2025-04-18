cmake_minimum_required(VERSION 3.8)
project(interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CanMsg.msg"
  "msg/EngineData.msg"
  "msg/PumpRpm.msg"
  "msg/Errors.msg"
  "msg/FuelAmbient.msg"
  "msg/GlowPlugs.msg"
  "msg/LastRunInfo.msg"
  "msg/NgReg.msg"
  "msg/Statistics.msg"
  "msg/SystemInfo.msg"
  "msg/SystemInfo2.msg"
  "msg/VoltageCurrent.msg"
  "srv/SendCanMessage.srv"
  "action/StarterTest.action"
  "action/Start.action"
  "action/Prime.action"
  "action/IgniterTest.action"
  "action/PumpTest.action"
  "action/ThrottleProfile.action"
  DEPENDENCIES std_msgs
)

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

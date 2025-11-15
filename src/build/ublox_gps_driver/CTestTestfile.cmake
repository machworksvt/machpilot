# CMake generated Testfile for 
# Source directory: /workspace/src/embedded_system/ublox_gps_driver
# Build directory: /workspace/src/build/ublox_gps_driver
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(test_checksum "/usr/bin/python3" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/workspace/src/build/ublox_gps_driver/test_results/ublox_gps_driver/test_checksum.gtest.xml" "--package-name" "ublox_gps_driver" "--output-file" "/workspace/src/build/ublox_gps_driver/ament_cmake_gtest/test_checksum.txt" "--command" "/workspace/src/build/ublox_gps_driver/test_checksum" "--gtest_output=xml:/workspace/src/build/ublox_gps_driver/test_results/ublox_gps_driver/test_checksum.gtest.xml")
set_tests_properties(test_checksum PROPERTIES  LABELS "gtest" REQUIRED_FILES "/workspace/src/build/ublox_gps_driver/test_checksum" TIMEOUT "60" WORKING_DIRECTORY "/workspace/src/build/ublox_gps_driver" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_gtest/cmake/ament_add_gtest_test.cmake;86;ament_add_test;/opt/ros/humble/share/ament_cmake_gtest/cmake/ament_add_gtest.cmake;93;ament_add_gtest_test;/workspace/src/embedded_system/ublox_gps_driver/CMakeLists.txt;101;ament_add_gtest;/workspace/src/embedded_system/ublox_gps_driver/CMakeLists.txt;0;")
subdirs("gtest")

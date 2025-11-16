cmake_minimum_required(VERSION 3.8)
project(ublox_gps_test)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Include directories
include_directories(
    ../include
    ../../dependencies/include
)

# Compile UART C library
add_library(uart STATIC
    ../../dependencies/src/uart.c
)

# Compile ublox_messages
add_library(ublox_messages STATIC
    ../src/ublox_messages.cpp
)

# Compile ublox_parser
add_library(ublox_parser STATIC
    ../src/ublox_parser.cpp
)
target_link_libraries(ublox_parser ublox_messages)

# Build test_parser_gps executable
add_executable(test_parser_gps
    test_parser_gps.cpp
)

# Build test_uart_interface executable
add_executable(test_uart_interface
    test_uart_interface.cpp
    ../src/uart_interface.cpp
)

target_link_libraries(test_uart_interface
    uart
)

target_link_libraries(test_parser_gps
    uart
    ublox_messages
    ublox_parser
)

# Installation (optional)
install(TARGETS test_parser_gps test_uart_interface
    DESTINATION bin
)
#include <gtest/gtest.h>
#include "ublox_gps_driver/uart_interface.hpp"
#include "ublox_gps_driver/ublox_messages.hpp"
#include <thread>
#include <chrono>

TEST(UARTInterface, ReadRawBytes) {
    // CHANGE THIS to your GPS device!
    const char* device = "/dev/ttyUSB0";
    
    ublox::UARTInterface uart(device);
    
    ASSERT_TRUE(uart.open()) << "Failed to open " << device;
    ASSERT_TRUE(uart.configure(38400)) << "Failed to configure UART";
    
    std::cout << "Reading raw bytes from GPS for 3 seconds..." << std::endl;
    
    uint8_t buffer[256];
    bool found_ubx = false;
    
    for (int i = 0; i < 30; i++) {
        ssize_t bytes = uart.read(buffer, sizeof(buffer));
        
        if (bytes > 0) {
            std::cout << "Read " << bytes << " bytes: ";
            for (ssize_t j = 0; j < std::min(bytes, (ssize_t)32); j++) {
                printf("%02X ", buffer[j]);
            }
            std::cout << std::endl;
            
            // Look for UBX sync bytes (0xB5 0x62)
            for (ssize_t j = 0; j < bytes - 1; j++) {
                if (buffer[j] == 0xB5 && buffer[j+1] == 0x62) {
                    std::cout << " Found UBX sync bytes at offset " << j << std::endl;
                    found_ubx = true;
                }
            }
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    uart.close();
    
    EXPECT_TRUE(found_ubx) << "Did not find UBX sync bytes (0xB5 0x62)";
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
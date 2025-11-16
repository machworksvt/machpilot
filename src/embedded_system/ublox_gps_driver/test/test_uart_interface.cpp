#include "ublox_gps_driver/uart_interface.hpp"
#include <iostream>
#include <iomanip>
#include <thread>
#include <chrono>

int main() {
    std::cout << "=== Testing UART Interface ===" << std::endl;
    
    // Create UART interface
    ublox::UARTInterface uart("/dev/ttyUSB0");
    
    std::cout << "Opening device..." << std::endl;
    if (!uart.open()) {
        std::cerr << "Failed to open UART!" << std::endl;
        return 1;
    }
    std::cout << "✓ Opened" << std::endl;
    
    std::cout << "Configuring 38400 baud..." << std::endl;
    if (!uart.configure(38400)) {
        std::cerr << "Failed to configure UART!" << std::endl;
        return 1;
    }
    std::cout << "Configured" << std::endl;
    
    std::cout << "\nFlushing buffer..." << std::endl;
    uint8_t flush_buf[256];
    for (int i = 0; i < 5; i++) {
        uart.read(flush_buf, sizeof(flush_buf));
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    std::cout << "✓ Flushed" << std::endl;
    
    std::cout << "\nReading 10 times..." << std::endl;
    uint8_t buffer[256];
    
    for (int loop = 0; loop < 10; loop++) {
        ssize_t bytes_read = uart.read(buffer, sizeof(buffer));
        
        std::cout << "Read #" << (loop + 1) << ": " << bytes_read << " bytes" << std::endl;
        
        if (bytes_read > 0) {
            std::cout << "  First 16 bytes: ";
            for (ssize_t i = 0; i < std::min((ssize_t)16, bytes_read); i++) {
                std::cout << std::hex << std::setw(2) << std::setfill('0') 
                         << (int)buffer[i] << " ";
            }
            std::cout << std::dec << std::endl;
            
            // Check if all zeros
            bool all_zeros = true;
            for (ssize_t i = 0; i < bytes_read; i++) {
                if (buffer[i] != 0) {
                    all_zeros = false;
                    break;
                }
            }
            
            if (all_zeros) {
                std::cout << "WARNING: All zeros!" << std::endl;
            }
        } else if (bytes_read == 0) {
            std::cout << "  No data available" << std::endl;
        } else {
            std::cout << "  Read error!" << std::endl;
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    uart.close();
    std::cout << "\n Test complete" << std::endl;
    
    return 0;
}
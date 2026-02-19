#include <iostream>
#include <thread>
#include <chrono>

extern "C" {
    #include "uart.h"
}

int main() {
    const char* device = "/dev/cu.usbserial-BG009G79";
    
    UARTInfo uart_info;
    uart_info.fd = -1;
    
    std::cout << "Opening " << device << "..." << std::endl;
    if (uart_open(&uart_info, device) != 0) {
        std::cerr << "Failed to open device" << std::endl;
        return 1;
    }
    
    std::cout << "Configuring 38400 baud..." << std::endl;
    if (uart_configure(&uart_info, 38400, 0, 1, 8, 0, 1) != 0) {
        std::cerr << "Failed to configure" << std::endl;
        return 1;
    }
    
    std::cout << "Reading for 5 seconds..." << std::endl;
    uint8_t buffer[256];
    
    for (int i = 0; i < 50; i++) {
        ssize_t bytes = uart_read(&uart_info, buffer, sizeof(buffer));
        
        if (bytes > 0) {
            std::cout << "Read " << bytes << " bytes: ";
            for (ssize_t j = 0; j < std::min((ssize_t)32, bytes); j++) {
                printf("%02X ", buffer[j]);
            }
            std::cout << std::endl;
            
            // Look for UBX sync
            for (ssize_t j = 0; j < bytes - 1; j++) {
                if (buffer[j] == 0xB5 && buffer[j+1] == 0x62) {
                    std::cout << "Found UBX sync bytes!" << std::endl;
                }
            }
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    uart_close(&uart_info);
    std::cout << "Done!" << std::endl;
    return 0;
}
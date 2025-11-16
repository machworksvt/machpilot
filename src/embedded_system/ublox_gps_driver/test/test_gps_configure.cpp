#include <iostream>
#include <thread>
#include <chrono>
#include <vector>

extern "C" {
    #include "uart.h"
}

// Calculate UBX checksum
void calculateChecksum(const uint8_t* data, size_t length, uint8_t& ck_a, uint8_t& ck_b) {
    ck_a = 0;
    ck_b = 0;
    for (size_t i = 0; i < length; i++) {
        ck_a = (ck_a + data[i]) & 0xFF;
        ck_b = (ck_b + ck_a) & 0xFF;
    }
}

// Build CFG-PRT message (configure UART for UBX only)
std::vector<uint8_t> buildConfigurePort() {
    std::vector<uint8_t> msg;
    
    msg.push_back(0xB5); // Sync 1
    msg.push_back(0x62); // Sync 2
    msg.push_back(0x06); // Class: CFG
    msg.push_back(0x00); // ID: PRT
    msg.push_back(20);   // Length low
    msg.push_back(0);    // Length high
    
    // Payload (20 bytes)
    msg.push_back(1);    // Port ID (1 = UART)
    msg.push_back(0);    // Reserved
    msg.push_back(0);    // txReady
    msg.push_back(0);
    
    // UART mode (8N1)
    msg.push_back(0xC0);
    msg.push_back(0x08);
    msg.push_back(0x00);
    msg.push_back(0x00);
    
    // Baud rate (38400 = 0x00009600)
    msg.push_back(0x00);
    msg.push_back(0x96);
    msg.push_back(0x00);
    msg.push_back(0x00);
    
    // Input protocols (UBX only = 0x0001)
    msg.push_back(0x01);
    msg.push_back(0x00);
    
    // Output protocols (UBX only = 0x0001)
    msg.push_back(0x01);
    msg.push_back(0x00);
    
    // Flags
    msg.push_back(0x00);
    msg.push_back(0x00);
    
    // Reserved
    msg.push_back(0x00);
    msg.push_back(0x00);
    
    // Calculate checksum (over class, id, length, payload)
    uint8_t ck_a, ck_b;
    calculateChecksum(msg.data() + 2, msg.size() - 2, ck_a, ck_b);
    
    msg.push_back(ck_a);
    msg.push_back(ck_b);
    
    return msg;
}

int main() {
    const char* device = "/dev/cu.usbserial-BG009G79";
    
    UARTInfo uart_info;
    uart_info.fd = -1;
    
    std::cout << "Opening " << device << "..." << std::endl;
    if (uart_open(&uart_info, device) != 0) {
        std::cerr << "Failed to open" << std::endl;
        return 1;
    }
    
    std::cout << "Configuring 38400 baud..." << std::endl;
    if (uart_configure(&uart_info, 38400, 0, 1, 8, 0, 1) != 0) {
        std::cerr << "Failed to configure" << std::endl;
        return 1;
    }
    
    // Build and send CFG-PRT command
    std::cout << "\nSending CFG-PRT to enable UBX protocol..." << std::endl;
    auto cfg_prt = buildConfigurePort();
    
    std::cout << "Sending " << cfg_prt.size() << " bytes: ";
    for (auto b : cfg_prt) {
        printf("%02X ", b);
    }
    std::cout << std::endl;
    
    ssize_t written = uart_write(&uart_info, cfg_prt.data(), cfg_prt.size());
    std::cout << "Wrote " << written << " bytes" << std::endl;
    
    // Wait for GPS to process
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    // Now read data - should be UBX format!
    std::cout << "\nReading GPS data (should be UBX now)..." << std::endl;
    uint8_t buffer[256];
    bool found_ubx = false;
    
    for (int i = 0; i < 30; i++) {
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
                    std::cout << "FOUND UBX SYNC BYTES" << std::endl;
                    found_ubx = true;
                }
            }
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    uart_close(&uart_info);
    
    if (found_ubx) {
        std::cout << "\nSUCCESS: GPS is now outputting UBX protocol!" << std::endl;
    } else {
        std::cout << "\n Still seeing NMEA, not UBX" << std::endl;
    }
    
    return 0;
}
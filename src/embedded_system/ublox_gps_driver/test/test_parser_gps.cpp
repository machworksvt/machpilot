#include <iostream>
#include <thread>
#include <chrono>
#include <vector>

extern "C" {
    #include "uart.h"
}

#include "ublox_gps_driver/ublox_parser.hpp"

void calculateChecksum(const uint8_t* data, size_t length, uint8_t& ck_a, uint8_t& ck_b) {
    ck_a = 0;
    ck_b = 0;
    for (size_t i = 0; i < length; i++) {
        ck_a = (ck_a + data[i]) & 0xFF;
        ck_b = (ck_b + ck_a) & 0xFF;
    }
}

std::vector<uint8_t> buildEnableNavPVT() {
    std::vector<uint8_t> msg;
    
    msg.push_back(0xB5);
    msg.push_back(0x62);
    msg.push_back(0x06);  // CFG
    msg.push_back(0x01);  // MSG
    msg.push_back(3);
    msg.push_back(0);
    
    msg.push_back(0x01);  // NAV
    msg.push_back(0x07);  // PVT
    msg.push_back(1);     // Rate
    
    uint8_t ck_a, ck_b;
    calculateChecksum(msg.data() + 2, msg.size() - 2, ck_a, ck_b);
    msg.push_back(ck_a);
    msg.push_back(ck_b);
    
    return msg;
}

int main() {
    const char* device = "/dev/ttyUSB0";
    
    UARTInfo uart_info;
    uart_info.fd = -1;
    
    std::cout << "Opening GPS..." << std::endl;
    if (uart_open(&uart_info, device) != 0) {
        std::cerr << "Failed to open" << std::endl;
        return 1;
    }
    
    if (uart_configure(&uart_info, 38400, 0, 1, 8, 0, 1) != 0) {
        std::cerr << "Failed to configure" << std::endl;
        return 1;
    }
    
    // Flush any existing data
    std::cout << "Flushing buffer..." << std::endl;
    uint8_t flush_buf[256];
    for (int i = 0; i < 10; i++) {
        uart_read(&uart_info, flush_buf, sizeof(flush_buf));
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    // Enable NAV-PVT
    std::cout << "Enabling NAV-PVT..." << std::endl;
    auto enable_msg = buildEnableNavPVT();
    uart_write(&uart_info, enable_msg.data(), enable_msg.size());
    
    // Wait and flush ACK response
    std::cout << "Waiting for ACK..." << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    for (int i = 0; i < 10; i++) {
        ssize_t bytes = uart_read(&uart_info, flush_buf, sizeof(flush_buf));
        if (bytes > 0) {
            std::cout << "Flushed " << bytes << " bytes" << std::endl;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    // Now start parsing
    std::cout << "\n=== Starting to parse NAV-PVT ===" << std::endl;
    
    ublox::UbloxParser parser;
    int message_count = 0;
    
    parser.setNavPVTCallback([&](const ublox::NavPVT& pvt) {
        message_count++;
        
        std::cout << "\nNAV-PVT #" << message_count << std::endl;
        std::cout << "Time: " << (int)pvt.hour << ":" 
                  << (int)pvt.min << ":" << (int)pvt.sec << std::endl;
        std::cout << "Fix: " << (int)pvt.fixType << " | Sats: " << (int)pvt.numSV << std::endl;
        
        double lat = pvt.lat * 1e-7;
        double lon = pvt.lon * 1e-7;
        double alt = pvt.hMSL * 1e-3;
        
        std::cout << "Lat: " << lat << "° | Lon: " << lon << "°" << std::endl;
        std::cout << "Alt: " << alt << " m | Acc: " << pvt.hAcc * 1e-3 << " m" << std::endl;
    });
    
    uint8_t buffer[256];
    for (int i = 0; i < 100; i++) {
        ssize_t bytes = uart_read(&uart_info, buffer, sizeof(buffer));
        
        if (bytes > 0) {
            for (ssize_t j = 0; j < bytes; j++) {
                parser.processByte(buffer[j]);
            }
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    uart_close(&uart_info);
    
    std::cout << "\nTotal: " << message_count << " messages" << std::endl;
    
    if (message_count > 0) {
        std::cout << "SUCCESS!" << std::endl;
        return 0;
    } else {
        std::cout << "No messages" << std::endl;
        return 1;
    }
}
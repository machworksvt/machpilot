#pragma once

#include <string>
#include <vector>
#include <cstdint>

extern "C" {
    // need to change path env
    #include <uart.h>
}

namespace ublox {

class UARTInterface {
    public: 
        explicit UARTInterface(const std::string& device_path);
        ~UARTInterface();

        bool open();
        bool configure(int baud_rate = 38400);
        void close();

        ssize_t write(const uint8_t* data, size_t size);
        ssize_t read(uint8_t* buffer, size_t size);

        // Non-copyable
        UARTInterface(const UARTInterface&) = delete;
        UARTInterface& operator = (const UARTInterface&) = delete;

    private:
    std::string device_path_;
    UARTInfo uart_info_;
    bool is_open_{false};

};

}


#include "ublox_gps_driver/uart_interface.hpp"
#include <stdexcept>
#include <ostream>
#include <iostream>


namespace ublox {

    UARTInterface::UARTInterface(const std::string& device_path)
        : device_path_(device_path) {
            uart_info_.fd = -1;
        }

    UARTInterface::~UARTInterface() {
            if (is_open_) {
                close();
            }
        }
    
    bool UARTInterface::open() {
        if (uart_open(&uart_info_, device_path_.c_str()) != 0) {
            return false;
        }
        is_open_ = true;
        return true;
    }

    bool UARTInterface::configure(int baud_rate) {
        if (!is_open_) {
            return false;
        }

        if (uart_configure(&uart_info_, baud_rate, 0, 1, 8, 0, 1) != 0) {
            return false;
        }

        return true;
    }

    void UARTInterface::close() {
        if (is_open_) {
            uart_close(&uart_info_);
            is_open_ = false;
        }

    }

    ssize_t UARTInterface::write(const uint8_t* data, size_t size) {
        if (!is_open_) {
            return -1;
        }
        return uart_write(&uart_info_, data, size);
    }

    ssize_t UARTInterface::read(uint8_t* buffer, size_t size) { 
        if (!is_open_) {
            return -1;
        }
        return uart_read(&uart_info_, buffer, size);
    }

}
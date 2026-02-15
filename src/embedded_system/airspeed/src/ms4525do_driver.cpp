#include "ms4525do_driver.h"

#include <cstdlib>
#include <cerrno>
#include <stdexcept>
#include <iostream>
#include <chrono>
#include <thread>


float offset = 0.0;
bool calibFlag = false;

// int main() {
//     MS4525DO ms = MS4525DO(0x28);
//     while(1) {
//     	ms.readDF4();
    
//     	std::cout << ms.data_.press << std::endl;
//     	std::cout << ms.data_.temp << std::endl;
    
//     }
    
// }


MS4525DO::MS4525DO(const char *bus_path, uint8_t bus_num, uint16_t addr) {
    i2c_info_.bus_num = bus_num;
    i2c_info_.address = addr;

    if (i2c_init(&i2c_info_, bus_path, bus_num) != 0) {
        perror("MS4525DO: bus initialization failed");
        exit(1);
    }
    
}

MS4525DO::~MS4525DO() {
    i2c_deinit(&i2c_info_);
}

uint8_t MS4525DO::readMeasureRequest() {
    if (i2c_read_cmd(&i2c_info_, NULL, 0)) {
        perror("MS4525DO: read error");
    close(i2c_info_.fd);
    }
}

uint8_t MS4525DO::readPressure() {

    uint8_t data[2];
    if (i2c_read(&i2c_info_, 0, 2, data)) {
        std::cerr << "Pitot: 2-read error" << std::endl;
        return 1;
    }

    uint16_t pvalue = (data[0] << 8) + data[1];
    uint8_t status = pvalue >> 14;
    pvalue = pvalue & 0x3fff;

    data_.status = status;

    bool doWrite = statusMessages(status);
    
    if (doWrite) {
        data_.pressure = ((pvalue - 0x3FFF * MIN) * (MAXP - MINP) / ((MAX - MIN) * 0x3FFF)) + MINP;

        if (calibFlag) {
            data_.pressure -= p_offset_;
        }
    }
    
    return 0;
}

uint8_t MS4525DO::readPressureAndTemp() {
    uint8_t data[3];
    if (i2c_read(&i2c_info_, 0, 3, data)) {
        std::cerr << "Pitot: 3-read error" << std::endl;
        return 1;
    }

    uint16_t pvalue = (data[0] << 8) + data[1];
    uint8_t status = pvalue >> 14;
    pvalue = pvalue & 0x3fff;

    data_.status = status;

    bool doWrite = statusMessages(status);
    
    if (doWrite) {
        data_.pressure = ((pvalue - 0x3FFF * MIN) * (MAXP - MINP) / ((MAX - MIN) * 0x3FFF)) + MINP;

        data_.temp = ((float)data[2] * (MAXT - MINT) / 0xFF) + MINT;

        if (calibFlag) {
            data_.pressure -= p_offset_;
            data_.temp -= t_offset_;
        }
    }
    
    return 0;
}

uint8_t MS4525DO::readPressureAndTempHD() {
    uint8_t data[4];
    if (i2c_read(&i2c_info_, 0, 4, data)) {
        std::cerr << "Pitot: 4-read error" << std::endl;
        return 1;
    }

    uint16_t pvalue = (data[0] << 8) + data[1];
    uint8_t status = pvalue >> 14;
    pvalue = pvalue & 0x3fff;
    
    uint16_t tvalue = (data[2] << 3) + (data[3] >> 5);

    data_.status = status;

    bool doWrite = statusMessages(status);
    
    if (doWrite) {
        data_.pressure = ((pvalue - 0x3FFF * MIN) * (MAXP - MINP) / ((MAX - MIN) * 0x3FFF)) + MINP;

        data_.temp = ((float)tvalue * (MAXT - MINT) / 0x7FF) + MINT;

        if (calibFlag) {
            data_.pressure -= p_offset_;
            data_.temp -= t_offset_;
        }
    }
    
    return 0;
}

bool MS4525DO::statusMessages(uint8_t status) {

    switch (status) {
    case STATUS_NORMAL:
        return true;
    case STATUS_RESERVED:
        return true;
    case STATUS_STALE:
        perror("stale data, read again");
        return false;
    case STATUS_ERROR:
        throw std::runtime_error("read status error, perform power-on reset");
        return false;
    }
    return false;
}


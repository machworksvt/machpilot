#include "ms4525do_driver.h"

#include <cstdlib>
#include <cerrno>
#include <stdexcept>
#include <iostream>
#include <chrono>
#include <thread>

extern "C" {
    #include <linux/kernel.h>
}

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
    _i2c_info.bus_num = bus_num;
    _i2c_info.address = addr;

    if (MS4525DO::init(bus_num, bus_path) != 0) {
        perror("MS4525DO: bus initialization failed");
        exit(1);
    }
    
}

MS4525DO::~MS4525DO() {
    close(_i2c_info.fd);
}

uint8_t MS4525DO::init(uint8_t bus_num, const char *bus_path) {
    
    if (i2c_init(&_i2c_info, bus_path, bus_num) != 0) {
        perror("MS4525DO: bus initialization failed");
        exit(1);
    }

    return 0;
}

uint8_t MS4525DO::readMR() {

    if (i2c_read(&_i2c_info, 0, 0, NULL)) {
        std::cerr << "read error" << std::endl;
        return 2;
    }

    if (usleep(1000000 / MAX_POLLING)) {
        perror("MS4525DO: readMR delay interrupted");
        return 1;
    }

    return 0;
}

uint8_t MS4525DO::readDF2() {

    uint8_t data[2];
    if (i2c_read(&_i2c_info, 0, 2, data)) {
        std::cerr << "Pitot: 2-read error" << std::endl;
        return 1;
    }

    uint16_t pvalue = (data[0] << 8) + data[1];
    pvalue = pvalue & 0x3fff;
    uint8_t status = pvalue >> 14;

    _data.status = status;

    bool doWrite = statusMessages(status);
    
    if (doWrite) {
        _data.pressure = ((pvalue - 0x3FFF * MIN) * (MAXP - MINP) / ((MAX - MIN) * 0x3FFF)) + MINP;

        if (calibFlag) {
            _data.pressure -= p_offset;
        }
    }
    
    return 0;
}

uint8_t MS4525DO::readDF3() {
    uint8_t data[3];
    if (i2c_read(&_i2c_info, 0, 3, data)) {
        std::cerr << "Pitot: 3-read error" << std::endl;
        return 1;
    }

    uint16_t pvalue = (data[0] << 8) + data[1];
    pvalue = pvalue & 0x3fff;
    uint8_t status = pvalue >> 14;

    _data.status = status;

    bool doWrite = statusMessages(status);
    
    if (doWrite) {
        _data.pressure = ((pvalue - 0x3FFF * MIN) * (MAXP - MINP) / ((MAX - MIN) * 0x3FFF)) + MINP;

        _data.temp = ((float)data[2] * (MAXT - MINT) / 0xFF) + MINT;

        if (calibFlag) {
            _data.pressure -= p_offset;
            _data.temp -= t_offset;
        }
    }
    
    return 0;
}

uint8_t MS4525DO::readDF4() {
    uint8_t data[4];
    if (i2c_read(&_i2c_info, 0, 4, data)) {
        std::cerr << "Pitot: 4-read error" << std::endl;
        return 1;
    }

    uint16_t pvalue = (data[0] << 8) + data[1];
    pvalue = pvalue & 0x3fff;
    uint8_t status = pvalue >> 14;

    uint16_t tvalue = (data[2] << 3) + (data[3] >> 5);

    _data.status = status;

    bool doWrite = statusMessages(status);
    
    if (doWrite) {
        _data.pressure = ((pvalue - 0x3FFF * MIN) * (MAXP - MINP) / ((MAX - MIN) * 0x3FFF)) + MINP;

        _data.temp = ((float)tvalue * (MAXT - MINT) / 0x7FF) + MINT;

        if (calibFlag) {
            _data.pressure -= p_offset;
            _data.temp -= t_offset;
        }
    }
    
    return 0;
}

bool MS4525DO::statusMessages(uint8_t status) {

    switch (status) {
    case 0:
        return true;
    case 1:
        return true;
    case 2:
        perror("stale data, read again");
        return false;
    case 3:
        throw std::runtime_error("read status error, perform power-on reset");
        return false;
    }
    return false;
}


#include "ms4525do_driver.h"

#include <cstdlib>
#include <cerrno>
#include <stdexcept>
#include <iostream>
#include <chrono>
#include <thread>



extern "C" {
    #include <linux/i2c.h>
    #include <linux/i2c-dev.h>
    #include <i2c/smbus.h>
    #include <fcntl.h>
    #include <sys/ioctl.h>
    #include <unistd.h>
    #include <linux/kernel.h>
}

/**
int main() {
    MS4525DO ms = MS4525DO(0x28);
    while(1) {
    	ms.readDF4();
    
    	std::cout << ms.data_.press << std::endl;
    	std::cout << ms.data_.temp << std::endl;
    
    }
    
}
*/

MS4525DO::MS4525DO(int addr) {
    file_ = open(I2C_FILE_PATH, O_RDWR);
    addr_ = addr;

    data_.press = 0;
    data_.temp = 0;
    data_.status = 0;

    init();
}

MS4525DO::~MS4525DO() {
    close(file_);
}

uint8_t MS4525DO::init() {
    
    if (file_ < 0) {
        close(file_);
        exit(1);
    }

    if (ioctl(file_, I2C_SLAVE, addr_) < 0) {
        close(file_);
        exit(2);
    }

    return 0;
}

uint8_t MS4525DO::readMR() {
    i2c_rdwr_ioctl_data pkt;
    i2c_msg msg[1];
    msg[0].addr = addr_;
    msg[0].flags = I2C_M_RD;
    msg[0].len = 0;
    msg[0].buf = NULL;

    pkt.msgs = msg;
    pkt.nmsgs = 1;

    if (ioctl(file_, I2C_RDWR, &pkt) < 0) {
        std::cerr << "measure start error" << std::endl;
        // don't stop on error, readMR is not necessary
    }

    return 0;
}

uint8_t MS4525DO::readDF2() {
    uint8_t records[2];
    if (read(file_, records, 2) != 2) {
        std::cerr << "read error" << std::endl;
        return 2;
    }

    uint16_t pvalue = (records[0] << 8) + records[1];

    uint8_t status = pvalue >> 14;
    data_.status = status;

    pvalue = pvalue & 0x3fff;

    bool doWrite = statusMessages(data_.status);

    if (doWrite) {
        float pressure = ((pvalue - 0x3FFF * MIN) * (MAXP - MINP) / ((MAX - MIN) * 0x3FFF)) + MINP;
        data_.press = pressure;
    }
    
    return 0;
}

uint8_t MS4525DO::readDF3() {
    uint8_t records[3];
    if (read(file_, records, 3) != 3) {
        std::cerr << "read error" << std::endl;
        return 2;
    }

    uint16_t pvalue = (records[0] << 8) + records[1];

    uint8_t status = pvalue >> 14;
    data_.status = status;

    pvalue = pvalue & 0x3fff;

    uint16_t tvalue = records[2] << 3;

    bool doWrite = statusMessages(data_.status);

    if (doWrite) {
        float pressure = ((pvalue - 0x3FFF * MIN) * (MAXP - MINP) / ((MAX - MIN) * 0x3FFF)) + MINP;
        data_.press = pressure;

        float tempC = ((float)tvalue * (MAXT - MINT) / 0x7FF) + MINT;
        data_.temp = tempC;
    }
    
    return 0;
}

uint8_t MS4525DO::readDF4() {
    uint8_t records[4];
    if (read(file_, records, 4) != 4) {
        std::cerr << "read error" << std::endl;
        return 2;
    }

    uint16_t pvalue = (records[0] << 8) | records[1];

    uint8_t status = pvalue >> 14;
    data_.status = status;

    pvalue = pvalue & 0x3fff;

    uint16_t tvalue = (records[2] << 3) | (records[3] >> 5);

    bool doWrite = statusMessages(data_.status);

    if (doWrite) {
        float pressure = ((pvalue - 0x3FFF * MIN) * (MAXP - MINP) / ((MAX - MIN) * 0x3FFF)) + MINP;
        data_.press = pressure;

        float tempC = ((float)tvalue * (MAXT - MINT) / 0x7FF) + MINT;
        data_.temp = tempC;
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
        std::cerr << "stale data" << std::endl;
        return false;
    case 3:
        throw std::runtime_error("read status error, perform power-on reset");
        return false;
    }
    return false;
}

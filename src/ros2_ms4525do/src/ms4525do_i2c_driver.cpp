#include "ros2_ms4525do/ms4525do_i2c_driver.hpp"

namespace ms4525do {

MS4525DOI2CDriver::MS4525DOI2CDriver(std::string device_, int address_) {
    device = device_;
    address_read = (address_ << 1) + 1;
    address_write = address_;
}

bool MS4525DOI2CDriver::init() {
    file_ = open(device.c_str(), O_RDWR);

    if (ioctl(file_, I2C_SLAVE, address_read) < 0) {
        throw std::runtime_error("i2c device open failed");
        return false;
    }

    return true;
}

bool MS4525DOI2CDriver::writeMR() {
    // #TODO implement this better, for low power mode.
    // this should be called in init().
    if (_i2c_smbus_read_byte(file_) < 0) {
        throw std::runtime_error("write request error");
        return false;
    }

    return true;
}

bool MS4525DOI2CDriver::readMR() {
    // #TODO implement this better, for low power mode.
    // this should be called before every measurement
    if (_i2c_smbus_read_byte(file_) < 0) {
        throw std::runtime_error("request error");
        return false;
    }

    return true;
}

bool MS4525DOI2CDriver::readDF2() {
    uint16_t record;

    if (_i2c_smbus_read_i2c_block_data(file_, address_read, 0x02, (uint8_t*)&record) != 0x02) {
        throw std::runtime_error("pressure read error");
        return false;
    }

    uint8_t status = (record >> 14) & 3; // get status from data
    data_.status = status;

    record = record & 0x3fff; // remove status bits

    bool doWrite = statusMessages(data_.status);

    if (doWrite) {
        data_.press_raw = record;
    }
    else {
        data_.press_raw = 0;
    }

    return doWrite;
}

bool MS4525DOI2CDriver::readDF3() {
    uint32_t record;

    if (_i2c_smbus_read_i2c_block_data(file_, address_read, 0x03, (uint8_t*)&record) != 0x03) {
        throw std::runtime_error("pressure read error");
        return false;
    }

    uint8_t status = (record >> 22) & 3; // get status from data
    data_.status = status;

    record = record & 0x3FFFFF; // remove status bits

    bool doWrite = statusMessages(data_.status);

    if (doWrite) {
        data_.press_raw = record >> 8;
        data_.temp_raw = record & 0xff;
    }
    else {
        data_.press_raw = 0;
    }

    return doWrite;
}

bool MS4525DOI2CDriver::readDF4() {
    uint32_t record;

    if (_i2c_smbus_read_i2c_block_data(file_, address_read, 0x04, (uint8_t*)&record) != 0x04) {
        throw std::runtime_error("pressure read error");
        return false;
    }

    uint8_t status = (record >> 30) & 3; // get status from data
    data_.status = status;

    record = (record & 0x3fffffff) >> 5; // remove status and shift out unneeded bits

    bool doWrite = statusMessages(data_.status);

    if (doWrite) {
        data_.press_raw = record >> 11;
        data_.temp_raw = record & 0x7ff;
    }
    else {
        data_.press_raw = 0;
    }

    return doWrite;
}

void MS4525DOI2CDriver::getData(PITOTData &buffer) {
    buffer = data_;
}

bool statusMessages(uint8_t status) {
    switch (status) {
    case 0:
        return true;
    case 1:
        return true;
    case 2:
        std::cerr << "stale data" << std::endl;
        return true;
    case 3:
        throw std::runtime_error("read status error, perform power-on reset");
        return false;
    }

}

}
#ifndef _ms4525do_i2c_driver_dot_h
#define _ms4525do_i2c_driver_dot_h

#include <cstdlib>
#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <iostream>
#include <chrono>
#include <thread>

#include <linux/i2c-dev.h>
#include <linux/types.h>
#include <sys/ioctl.h>

#include <smbus_functions.h>

#define MS4525DO_ADDRESS_WRITE_A 0x28
#define MS4525DO_ADDRESS_WRITE_B 0x36
#define MS4525DO_ADDRESS_WRITE_C 0x46

#define MS4525DO_ADDRESS_READ_A 0x28
#define MS4525DO_ADDRESS_READ_B 0x36
#define MS4525DO_ADDRESS_READ_C 0x46

#define STATUS_NORMAL 0
#define STATUS_RESERVED 1
#define STATUS_STALE 2
#define STATUS_ERROR 3

namespace ms4525do {

typedef struct {
    uint8_t status;
    uint16_t temp_raw;
    uint16_t press_raw;
} PITOTData;

class MS4525DOI2CDriver {
public:
    MS4525DOI2CDriver(std::string device_, int address_);
    bool init();
    bool readMeasureReq();
    bool readDF2();
    bool readDF3();
    bool readDF4();
private:
    int file;
    std::string device;
    int address_read;
    int address_write;

    PITOTData data;
};

}

#endif

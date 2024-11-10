#ifndef _ms4525do_i2c_driver_dot_h
#define _ms4525do_i2c_driver_dot_h

#include <cstdlib>
#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <iostream>
#include <chrono>
#include <thread>

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/types.h>
#include <sys/ioctl.h>

#include <smbus_functions.h>

#define STATUS_NORMAL 0
#define STATUS_RESERVED 1
#define STATUS_STALE 2
#define STATUS_ERROR 3

#define MAX_POLLING 1666 // maximum polling frequency in Hz

#define AORB 0 // output type set 0 or 1
#define RANGE 001 // pressure measurement range in psi
#define DGVAC 1 // look at product code after range marker
#define INTERFACE 0x1 // I,J,K,S, then 0-9

#if AORB == 0
    #define MAXP 0.90
    #define MINP 0.10
#else
    #define MAXP 0.95
    #define MINP 0.5
#endif

namespace ms4525do {

typedef struct {
    uint8_t status;
    uint16_t temp_raw;
    uint16_t press_raw;
} PITOTData;

class MS4525DOI2CDriver {
public:
    MS4525DOI2CDriver(std::string device, int address);
    bool init();
    bool writeMR();
    bool readMR();
    bool readDF2();
    bool readDF3();
    bool readDF4();
    void getData(PITOTData &buffer);

    std::string device;
    int address_read;
    int address_write;
private:
    int file_;
    PITOTData data_;
};

}

#endif

#include "adxl345_driver.h"

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



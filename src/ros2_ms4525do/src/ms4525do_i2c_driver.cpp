#include "ros2_ms4525do/ms4525do_i2c_driver.hpp"

namespace ms4525do {

MS4525DOI2CDriver::MS4525DOI2CDriver(std::string device_, int address_) {
    device = device_;
    address_read = (address_ << 1) + 1;
    address_write = address_;
}


}
#include <bno055_driver.h>

BNO055::BNO055(const char *bus_path, uint8_t bus_num, uint8_t addr) {
    i2c_info_.bus_num = bus_num;
    i2c_info_.address = addr;

    if (this->init(bus_path, bus_num) != 0) {
        perror("BNO055: initialization failed");
        exit(1);
    }
}

BNO055::~BNO055() {
    close(i2c_info_.fd);
}

int BNO055::init(const char *bus_path, uint8_t bus_num) {
    
    if (i2c_init(&i2c_info_, bus_path, bus_num) != 0) {
        perror("BNO055: i2c initialization failed");
        exit(1);
    }

    return 0;
}
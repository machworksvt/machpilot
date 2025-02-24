#include "pca9685_driver.h"

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
    PCA9685 pca = PCA9685(0x40);

    pca.reset();

    float angles[16] = {10, 10, 10, 10,
                        10, 10, 10, 10, 
                        10, 10, 10, 10, 
                        10, 10, 10, 10};
    
    for (int i = 0; i < 16; i++) {
        pca.angles_[i] = angles[i];
    }
    
    pca.pwm_set_all();
    std::cout << "off_high" << std::endl;
}
*/

PCA9685::PCA9685(int addr) {
    file_ = open(I2C_FILE_PATH, O_RDWR);
    addr_ = addr;

    init();
}

PCA9685::~PCA9685() {
    close(file_);
}

uint8_t PCA9685::init() {
    if (file_ < 0) {
        close(file_);
        std::cout << "file open failed" << std::endl;
        exit(1);
    }

    if (ioctl(file_, I2C_SLAVE, addr_) < 0) {
        close(file_);
        std::cout << "ioctl init error" << std::endl;
        exit(2);
    }

    return 0;
}

void PCA9685::reset() {

    uint8_t modes[2];
    if (read(file_, modes, 2) < 2) {
        std::cout << "read error" << std::endl;
        return;
    }

    if (modes[0] & 0b00010000) {
        std::cout << "chip in sleep mode, resettng" << std::endl;
        uint8_t mode1[1] = {0b10000000};
        write(file_, mode1, 1);
    }

    if (modes[0] & 0b10000000) {
        std::cout << "restart in progress, don't write" << std::endl;
    }

    if (modes[0] & 0b00010000) {
        std::cout << "auto-increment enabled, disabling" << std::endl;
        uint8_t mode1[1] = {0b10010000};
        write(file_, mode1, 1);
    }

}

void PCA9685::change_freq(int freq) {

}

void PCA9685::sleep(int dur) {

}

uint8_t PCA9685::pwm_set(uint8_t ch) {

    float dc = ((MAXDC - MINDC) * angles_[ch] / MOTORSPAN + MINDC) / 4096.0;

    if (dc < 0) {
        std::cout << "negative duty cycle entered, clipped" << std::endl;
        dc = 0.0;
    }
    else if (dc > 1) {
        std::cout << "non-normalized duty cycle entered, clipped" << std::endl;
        dc = 1.0;
    }

    uint8_t reg = 4 * ch + 6;

    uint16_t pw = (int)((dc) * 0xFFF);
    
    uint8_t off_high = pw >> 8;
    uint8_t off_low = pw & 0x0FF;

    uint8_t buf[2];

    for (int8_t j = 0; j < 4; j++) {
        buf[0] = reg + j;
        buf[1] = 0 + ((j == 2) * off_low + (j == 3) * off_high);

        if(write(file_, buf, sizeof(buf)) != sizeof(buf)) {
            std::cout << "pwm write error" << std::endl;
            return 1;
        }
    }
    
    return 0;
}

void PCA9685::pwm_read(uint8_t ch, uint16_t* data) {

}

uint8_t PCA9685::pwm_set_all() {
    for (int i = 0; i < 16; i++) {
        if (pwm_set(i) != 0) {
            printf("pwm setting failed on channel %d", i);
            return 1;
        }
    }

    return 0;
}


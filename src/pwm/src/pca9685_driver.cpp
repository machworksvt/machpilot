#include "pca9685_driver.h"

#include <time.h>
#include <cstdlib>
#include <cerrno>
#include <stdexcept>
#include <iostream>
#include <chrono>
#include <thread>

/**
int main() {
    PCA9685 pca = PCA9685("/dev/i2c-", 0, 0x40);

    pca.reset();

    double angles[16] = {10.0, 10.0, 10.0, 10.0,
                         10.0, 10.0, 10.0, 10.0, 
                         10.0, 10.0, 10.0, 10.0,
                         10.0, 10.0, 10.0, 10.0};
    
    
    pca.pwm_set_all(angles);
    std::cout << "off_high" << std::endl;
}
*/

PCA9685::PCA9685(const char *bus_path, uint8_t bus_num, uint16_t addr) {
    I2CInfo _i2c_info;
    _i2c_info.bus_num = bus_num;

    if (i2c_init(&_i2c_info, bus_path, bus_num) != 0) {
        perror("PCA9685: bus initialization failed");
        exit(1);
    }

    _i2c_info.address = addr;

}

PCA9685::~PCA9685() {
    close(i2c_info_.fd);
}

int PCA9685::reset() {
    // software reset by writing 0b00000110 to general call address 0x00
    uint8_t buf[1] = {0b00000110};

    if(i2c_write(&i2c_info_, PCA9685_MODE1, 1, buf)) {
        perror("PCA9685: reset write failed");
        return -1;
    }

    struct timespec rqtp;
    struct timespec rmtp;
    rqtp.tv_sec = 0;
    rqtp.tv_nsec = TBUF; // 4.7 us

    // wait 4.7 us for reset to complete
    if (nanosleep(&rqtp, &rmtp)) {
        perror("PCA9685: reset delay interrupted");
        return -1;
    }

    // read MODE1 register to check if RESTART is set
    uint8_t mode1[1];
    if(i2c_read(&i2c_info_, PCA9685_MODE1, 1, mode1)) {
        perror("PCA9685: reset read failed");
        return -1;
    }

    // if RESTART is set, clear SLEEP bit and wait 500 us for oscillator to stabilize
    if (mode1[0] & RESTART) {
        
        // if SLEEP bit is set, clear it and wait 500 us for oscillator to stabilize
        if (mode1[0] & SLEEP) {
            mode1[0] &= ~SLEEP; // clear SLEEP bit
            // write back to MODE1 register
            if(i2c_write(&i2c_info_, PCA9685_MODE1, 1, mode1)) {
                perror("PCA9685: sleep write failed");
                return -1;
            }
            // wait 500 us for oscillator to stabilize
            rqtp.tv_sec = 0;
            rqtp.tv_nsec = SBUF; // 500 us

            if (nanosleep(&rqtp, &rmtp)) {
                perror("PCA9685: sleep stabilization interrupted");
                return -1;
            }
        }

        // set RESTART bit to 1 to restart
        mode1[0] |= RESTART;
        // write back to MODE1 register
        if(i2c_write(&i2c_info_, PCA9685_MODE1, 1, mode1)) {
            perror("PCA9685: restart write failed");
            return -1;
        }
    }   

    return 0;
}

int PCA9685::set_angle(double angle_deg, uint8_t ch) {
    if (ch > 15) {
        std::cerr << "PCA9685: channel out of range" << std::endl;
        return -1;
    }

    if (angle_deg < 0 || angle_deg > MOTORSPAN) {
        std::cerr << "PCA9685: angle out of range" << std::endl;
        return -1;
    }

    // map angle to off time, no on delay
    uint16_t off_count = (int)MINDC + (int)((angle_deg / MOTORSPAN) * (MAXDC - MINDC));
    if (off_count > 4095) {
        perror("PCA9685: off_count out of range");
        off_count = 4095;
    }

    uint8_t data[4];
    data[0] = 0; // ON_L
    data[1] = 0; // ON_H
    data[2] = off_count & 0xFF; // OFF_L
    data[3] = (off_count >> 8) & 0x0F; // OFF_H

    // write to LEDn registers
    if(i2c_write(&i2c_info_, PCA9685_LED0_ON_L + 4 * ch, 4, data)) {
        perror("PCA9685: set_angle write failed");
        return -1;
    }

    return 0;
}

int PCA9685::set_angle_all(double *angles_deg) {
    // a flag to indicate if any channel failed
    bool failure_flag = false;

    // iterate through all channels and set angle
    for (int ch = 0; ch < 16; ch++) {

        if (set_angle(ch, angles_deg[ch])) {
            printf("PCA9685: set_angle_all failed at channel %d\n", ch);
            failure_flag = true;
            continue;
        }
    }

    return failure_flag ? -1 : 0;
}
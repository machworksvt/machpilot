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
    i2c_info_.bus_num = bus_num;

    if (i2c_init(&i2c_info_, bus_path, bus_num) != 0) {
        perror("PCA9685: bus initialization failed");
        exit(1);
    }

    i2c_info_.address = addr;

    // Initialize the PCA9685
    if (reset() != 0) {
        perror("PCA9685: reset failed during initialization");
        exit(1);
    }
    
    // Set frequency to default value (FREQ is defined in header)
    if (set_frequency(FREQ) != 0) {
        perror("PCA9685: frequency setup failed during initialization");
        exit(1);
    }
}

PCA9685::~PCA9685() {
    close(i2c_info_.fd);
}

int PCA9685::reset() {
    struct timespec rqtp;
    struct timespec rmtp;



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

int PCA9685::set_frequency(uint16_t freq) {
    // Calculate prescale value for desired frequency
    // PCA9685 internal oscillator is 25MHz
    // PWM frequency = 25MHz / (4096 * (prescale + 1))
    // prescale = (25MHz / (4096 * freq)) - 1
    uint8_t prescale = (uint8_t)(25000000.0 / (4096.0 * freq) - 1.0 + 0.5); // +0.5 for rounding
    
    // Read current MODE1 register
    uint8_t mode1[1];
    if(i2c_read(&i2c_info_, PCA9685_MODE1, 1, mode1)) {
        perror("PCA9685: frequency read MODE1 failed");
        return -1;
    }
    
    // Set SLEEP bit to enter sleep mode (required to change prescale)
    uint8_t sleep_mode = mode1[0] | SLEEP;
    if(i2c_write(&i2c_info_, PCA9685_MODE1, 1, &sleep_mode)) {
        perror("PCA9685: frequency sleep write failed");
        return -1;
    }
    
    // Write prescale value
    if(i2c_write(&i2c_info_, PCA9685_PRESCALE, 1, &prescale)) {
        perror("PCA9685: frequency prescale write failed");
        return -1;
    }
    
    // Restore MODE1 register (clear SLEEP bit)
    if(i2c_write(&i2c_info_, PCA9685_MODE1, 1, mode1)) {
        perror("PCA9685: frequency restore MODE1 failed");
        return -1;
    }
    
    // Wait for oscillator to stabilize
    struct timespec rqtp, rmtp;
    rqtp.tv_sec = 0;
    rqtp.tv_nsec = SBUF; // 500 us
    if (nanosleep(&rqtp, &rmtp)) {
        perror("PCA9685: frequency stabilization interrupted");
        return -1;
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

    // map angle to off time, no ON delay
    uint16_t off_count = (int)MINDC + (int)((angle_deg / MOTORSPAN) * (MAXDC - MINDC));
    if (off_count > 4095) {
        perror("PCA9685: off_count out of range");
        off_count = 4095;
    }

    // delays array, since duty cycle is the only concern, phase doesn't matter
    uint8_t data[4];
    data[0] = 0; // ON_L
    data[1] = 0; // ON_H
    data[2] = off_count & 0xFF; // OFF_L
    data[3] = (off_count >> 8) & 0x0F; // OFF_H

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

        if (set_angle(angles_deg[ch], ch)) {
            printf("PCA9685: set_angle_all failed at channel %d\n", ch);
            failure_flag = true;
            continue;
        }
    }

    return failure_flag ? -1 : 0;
}
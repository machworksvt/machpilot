#ifndef PCA9685_DRIVER_DOT_H
#define PCA9685_DRIVER_DOT_H

#include <cstring>
#include <cstdint>

extern "C" {
    #include <i2c.h>
}

#define TBUF 4700 // 4.7 us in ns, software reset delay
#define SBUF 500000 // 500 us in ns, oscillator stabilization delay

#define FREQ 350
#define MOTORSPAN 170.0
#define MAXDC 2022.0
#define MINDC 602.0

typedef enum {
    // Mode registers
    PCA9685_MODE1        = 0x00,  // Mode register 1
    PCA9685_MODE2        = 0x01,  // Mode register 2

    // Sub-address registers
    PCA9685_SUBADR1      = 0x02,  // I2C-bus subaddress 1
    PCA9685_SUBADR2      = 0x03,  // I2C-bus subaddress 2
    PCA9685_SUBADR3      = 0x04,  // I2C-bus subaddress 3

    // All-call address
    PCA9685_ALLCALLADR   = 0x05,

    PCA9685_LED0_ON_L    = 0x06,
    PCA9685_LED0_ON_H    = 0x07,
    PCA9685_LED0_OFF_L   = 0x08,
    PCA9685_LED0_OFF_H   = 0x09,

    PCA9685_LED15_ON_L   = 0x42,
    PCA9685_LED15_ON_H   = 0x43,
    PCA9685_LED15_OFF_L  = 0x44,
    PCA9685_LED15_OFF_H  = 0x45,

    // ALL_LED registers (write affects all channels)
    PCA9685_ALL_LED_ON_L  = 0xFA,
    PCA9685_ALL_LED_ON_H  = 0xFB,
    PCA9685_ALL_LED_OFF_L = 0xFC,
    PCA9685_ALL_LED_OFF_H = 0xFD,

    // Pre-scale register for PWM frequency
    PCA9685_PRESCALE       = 0xFE,

    // Test mode register (used for factory testing, not usually used)
    PCA9685_TESTMODE       = 0xFF
} PCA9685_Registers;


enum Mode1Bits {
    RESTART = 0x80,
    EXTCLK  = 0x40,
    AI      = 0x20,
    SLEEP   = 0x10,
    SUB1    = 0x08,
    SUB2    = 0x04,
    SUB3    = 0x02,
    ALLCALL = 0x01
};

enum Mode2Bits {
    INVRT   = 0x10,
    OCH     = 0x08,
    OUTDRV  = 0x04,
    OUTNE1  = 0x02,
    OUTNE0  = 0x01
};

class PCA9685 {
private:
public:
    I2CInfo i2c_info_;

    PCA9685(const char *bus_path, uint8_t bus_num, uint16_t addr);
    ~PCA9685();
    int reset();
    int set_angle(double angle_deg, uint8_t ch);
    int set_angle_all(double *angles_deg);
};

#endif
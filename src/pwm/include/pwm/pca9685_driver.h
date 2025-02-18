#ifndef _ms4525do_i2c_driver_dot_h
#define _ms4525do_i2c_driver_dot_h

#include <cstring>
#include <cstdint>

#define MODE1 0x00 // use only in init
#define MODE2 0x01 // don't touch
#define SUBADR1 0x02
#define SUBADR2 0x03
#define SUBADR3 0x04

#define ALLCALLADR 0x05 // use rarely

#define FREQ 350
#define MOTORSPAN 180.0


#define I2C_FILE_PATH "/dev/i2c-7"

class PCA9685 {
private:
    uint8_t init();
public:
    uint8_t addr_;
    int file_;
    float angles_[16];
    
    PCA9685(int addr);
    ~PCA9685();
    void reset();
    void change_freq(int freq);
    void sleep(int dur);
    uint8_t pwm_set(uint8_t ch);
    void pwm_read(uint8_t ch, uint16_t* data);
    uint8_t pwm_set_all();
};

#endif
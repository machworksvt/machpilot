#ifndef _adxl345_i2c_driver_dot_h
#define _adxl345_i2c_driver_dot_h

#include <cstring>
#include <cstdint>

#define I2C_FILE_PATH "/dev/i2c-7"

#define ADDR 0x1D

typedef enum registers {
    THRESH_TAP = 0x1D,
    OFSX = 0x1E,
    OFSY = 0x1F,
    OFSZ = 0x20,
    DUR = 0x21,
    LAT = 0x22,
    WIN = 0x23,
    THRESH_ACT = 0x24,
    THRESH_INACT = 0x25,
    TIME_INACT = 0x26,
    ACT_INACT_CTL = 0x27,
    THRESH_FF = 0x28,
    TIME_FF = 0x29,
    TAP_AXES = 0x2A,
    ACT_TAP_STATUS = 0x2B,
    BW_RATE = 0x2C,
    POWER_CTL = 0x2D,
    INT_ENABLE = 0x2E,
    INT_MAP = 0x2F,
    INT_SOURCE = 0x30,
    DATA_FORMAT = 0x31,
    DATAX0 = 0x32,
    DATAX1 = 0x33,
    DATAY0 = 0x34,
    DATAY1 = 0x35,
    DATAZ0 = 0x36,
    DATAZ1 = 0x37,
    FIFO_CTL = 0x38,
    FIFO_STATUS = 0x39
} registers;

typedef struct {
    // don't use this in the actual code, compensation is done internally
    // write these to 0x1E - 0x20 once found
    // minimum unit is 15.6 mg/LSB
    uint16_t ox;
    uint16_t oy;
    uint16_t oz;
} offsets;

typedef struct {
    uint16_t ax;
    uint16_t ay;
    uint16_t az;
} accelData;

class ADXL345 {
private:
    uint8_t init();
public:
    accelData data_;
    uint8_t addr_;
    int file_;

    ADXL345(int addr);
    ~ADXL345();
    uint8_t setOffsets();
    uint8_t setLATWIN();
    uint8_t setThresholds();
    uint8_t setTimes();
    uint8_t sleep(uint16_t dur);
    uint8_t configInts();
    uint8_t setDataFormat();
    uint8_t readData();
};

#endif
#ifndef MS4525DO_I2C_DRIVER_DOT_H
#define MS4525DO_I2C_DRIVER_DOT_H

#include <cstring>
#include <cstdint>

extern "C" {
    #include <i2c.h>
}


#define STATUS_NORMAL 0
#define STATUS_RESERVED 1
#define STATUS_STALE 2
#define STATUS_ERROR 3

#define MAX_POLLING 1666 // maximum polling frequency in Hz

#define AORB 0 // output type set 0 or 1
#define RANGE 001 // pressure measurement range in psi
#define DGVAC 1 // look at product code after range marker
#define INTERFACE 0x1 // I,J,K,S, then 0-9

#if AORB == 0 // max percentages based on type A or B
    #define MAX 0.90
    #define MIN 0.10
#else
    #define MAX 0.95
    #define MIN 0.5
#endif

#define MAXP 1.0
#define MINP -1.0

#define PSI2PA 6894.76 // conversion factor from psi to pascal

#define MINT -50.0
#define MAXT 150.0

class MS4525DO {
private:
    uint8_t init(uint8_t bus_num, const char *bus_path);
    bool statusMessages(uint8_t status);
public:
    I2CInfo _i2c_info;
    struct {
        float pressure;
        float temp;
        uint8_t status;
    } _data;
    bool calibFlag = false;
    float p_offset;
    float t_offset;
    
    MS4525DO(const char *bus_path, uint8_t bus_num, uint16_t addr);
    ~MS4525DO();
    uint8_t readMR();
    uint8_t readDF2();
    uint8_t readDF3();
    uint8_t readDF4();
};

#endif
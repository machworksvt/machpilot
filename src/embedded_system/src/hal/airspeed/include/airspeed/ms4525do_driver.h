#ifndef _ms4525do_i2c_driver_dot_h
#define _ms4525do_i2c_driver_dot_h

#include <cstring>
#include <cstdint>

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

#define MAXP 1
#define MINP -1

#define MINT -50
#define MAXT 150

#define I2C_FILE_PATH "/dev/i2c-7"

typedef struct {
    uint8_t status;
    float temp;
    float press;
} PITOTData;

class MS4525DO {
private:
    uint8_t init();
    bool statusMessages(uint8_t status);
public:
    PITOTData data_;
    uint8_t addr_;
    int file_;
    
    MS4525DO(int addr);
    ~MS4525DO();
    uint8_t readMR();
    uint8_t readDF2();
    uint8_t readDF3();
    uint8_t readDF4();
};

#endif
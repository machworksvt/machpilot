#pragma once

#include <cstring>
#include <cstdint>

extern "C" {
    #include <i2c-local.h>
}

constexpr int MAX_POLLING = 1666; // maximum polling frequency in Hz

constexpr int AORB = 0; // output type set 0 or 1
constexpr int RANGE = 001; // pressure measurement range in psi
constexpr int DGVAC = 1; // look at product code after range marker
constexpr int INTERFACE = 0x1; // I,J,K,S, then 0-9

#if AORB == 0 // max percentages based on type A or B
    constexpr double MAX = 0.90;
    constexpr double MIN = 0.10;
#else
    constexpr double MAX = 0.95;
    constexpr double MIN = 0.5;
#endif

constexpr double MAXP = 1.0;
constexpr double MINP = -1.0;

constexpr double PSI2PA = 6894.76; // conversion factor from psi to pascal

constexpr double MINT = -50.0;
constexpr double MAXT = 150.0;

enum data_status {
    STATUS_NORMAL = 0,
    STATUS_RESERVED = 1,
    STATUS_STALE = 2,
    STATUS_ERROR = 3
};
class MS4525DO {
private:
    bool statusMessages(uint8_t status);
public:
    I2CInfo i2c_info_;
    struct {
        double pressure;
        double temp;
        uint8_t status{0};
    } data_;
    
    bool calibFlag_{false};
    double p_offset_;
    double t_offset_;
    
    MS4525DO(const char *bus_path, uint8_t bus_num, uint16_t addr);
    ~MS4525DO();
    uint8_t readMeasureRequest();
    uint8_t readPressure();
    uint8_t readPressureAndTemp();
    uint8_t readPressureAndTempHD();
};

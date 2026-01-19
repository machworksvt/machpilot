#ifndef BNO055_DRIVER_H
#define BNO055_DRIVER_H

extern "C" {
    #include <i2c-local.h>
}


#define BNO055_ADDRESS_A 0x28
#define BNO055_ADDRESS_B 0x29

#define CONFIG_TO_OPRMODE_DELAY_MS 7
#define OPRMODE_TO_CONFIG_DELAY_MS 19

const struct accel_default_config {
    uint8_t pwr_mode = BNO055_PWR_MODE_NORMAL; // Normal power mode
    uint8_t range_g = 4; // ±4g
    float bandwidth_Hz = 62.5f; //  Hz
    uint8_t resolution_bits = 14; // bits
};

const struct gyro_default_config {
    uint8_t pwr_mode = BNO055_PWR_MODE_NORMAL; // Normal power mode
    uint16_t range_dps = 2000; // ±2000 degrees per second
    float bandwidth_Hz = 32.0f; // Hz
    uint8_t resolution_bits = 16; // bits
};

const struct mag_default_config {
    uint8_t pwr_mode = BNO055_PWR_MODE_NORMAL; // Normal power mode
    uint8_t output_data_rate_Hz = 20.0f; // 20 Hz
    uint8_t xy_repetition = 15; // 15 repetitions
    uint8_t z_repetition = 16; // 16 repetitions
    uint8_t resolution_bits_x = 13; // bits
    uint8_t resolution_bits_y = 13; // bits
    uint8_t resolution_bits_z = 15; // bits
};

enum bno055_status_t {
    BNO055_OK = 0,
    BNO055_ERROR = -1
};

enum bno055_opr_mode_t {
    BNO055_OPR_MODE_CONFIG = 0x00,
    BNO055_OPR_MODE_ACCONLY = 0x01,
    BNO055_OPR_MODE_MAGONLY = 0x02,
    BNO055_OPR_MODE_GYRONLY = 0x03,
    BNO055_OPR_MODE_ACCMAG = 0x04,
    BNO055_OPR_MODE_ACCGYRO = 0x05,
    BNO055_OPR_MODE_MAGGYRO = 0x06,
    BNO055_OPR_MODE_AMG = 0x07,
    BNO055_OPR_MODE_IMU = 0x08,
    BNO055_OPR_MODE_COMPASS = 0x09,
    BNO055_OPR_MODE_M4G = 0x0A,
    BNO055_OPR_MODE_NDOF_FMC_OFF = 0x0B,
    BNO055_OPR_MODE_NDOF = 0x0C
};

enum bno055_pwr_mode_t {
    BNO055_PWR_MODE_NORMAL = 0x00,
    BNO055_PWR_MODE_LOWPOWER = 0x01,
    BNO055_PWR_MODE_SUSPEND = 0x02
};

enum bno055_axis_remap_t{
    BNO055_AXIS_REMAP_X = 0x00,
    BNO055_AXIS_REMAP_Y = 0x01,
    BNO055_AXIS_REMAP_Z = 0x02
};

enum bno055_axis_sign_t {
    BNO055_AXIS_SIGN_POSITIVE = 0x00,
    BNO055_AXIS_SIGN_NEGATIVE = 0x01
};

enum bno055_accel_g_range_t {
    BNO055_ACCEL_G_RANGE_2G = 0x00,
    BNO055_ACCEL_G_RANGE_4G = 0x01,
    BNO055_ACCEL_G_RANGE_8G = 0x02,
    BNO055_ACCEL_G_RANGE_16G = 0x03
};

enum bno055_reg_t {
    BNO055_CHIP_ID_ADDR = 0x00,
    BNO055_ACC_ID_ADDR = 0x01,
    BNO055_MAG_ID_ADDR = 0x02,
    BNO055_GYR_ID_ADDR = 0x03,
    BNO055_ACC_CONFIG = 0x08,
    BNO055_MAG_CONFIG = 0x09,

    BNO055_GYR_CONFIG_0 = 0x0A,
    BNO055_GYR_CONFIG_1 = 0x0B,
    BNO055_ACC_SLEEP_CONFIG = 0x0C,
    BNO055_GYR_SLEEP_CONFIG = 0x0D,
    BNO055_MAG_SLEEP_CONFIG = 0x0E,
    BNO055_INT_MASK = 0x0F,
    BNO055_INT_ENABLE = 0x10,

    BNO055_OPR_MODE = 0x3D,
    BNO055_PWR_MODE = 0x3E,
    BNO055_EULER_H_LSB = 0x1A,
    BNO055_EULER_H_MSB = 0x1B,
    BNO055_EULER_R_LSB = 0x1C,
    BNO055_EULER_R_MSB = 0x1D,
    BNO055_EULER_P_LSB = 0x1E,
    BNO055_EULER_P_MSB = 0x1F,

    BNO055_TEMP_ADDR = 0x34,

    BNO055_UNIT_SEL = 0x3B,
    
    BNO055_AXIS_MAP_CONFIG = 0x41,
    BNO055_AXIS_MAP_SIGN = 0x42,

    BNO055_ACC_OFFSET_X_LSB = 0x55,
    BNO055_ACC_OFFSET_X_MSB = 0x56,
    BNO055_ACC_OFFSET_Y_LSB = 0x57,
    BNO055_ACC_OFFSET_Y_MSB = 0x58,
    BNO055_ACC_OFFSET_Z_LSB = 0x59,
    BNO055_ACC_OFFSET_Z_MSB = 0x5A,

    BNO055_GYR_OFFSET_X_LSB = 0x5B,
    BNO055_GYR_OFFSET_X_MSB = 0x5C,
    BNO055_GYR_OFFSET_Y_LSB = 0x5D,
    BNO055_GYR_OFFSET_Y_MSB = 0x5E,
    BNO055_GYR_OFFSET_Z_LSB = 0x5F,
    BNO055_GYR_OFFSET_Z_MSB = 0x60,

    BNO055_MAG_OFFSET_X_LSB = 0x61,
    BNO055_MAG_OFFSET_X_MSB = 0x62,
    BNO055_MAG_OFFSET_Y_LSB = 0x63,
    BNO055_MAG_OFFSET_Y_MSB = 0x64,
    BNO055_MAG_OFFSET_Z_LSB = 0x65,
    BNO055_MAG_OFFSET_Z_MSB = 0x66,
};

class BNO055 {
public:
    BNO055(const char *bus_path, uint8_t bus_num, uint8_t addr);
    ~BNO055();

    int init(const char *bus_path, uint8_t bus_num);
    int set_operation_mode(int mode);
    int calibrate_accel();
    int calibrate_gyro();
    int calibrate_mag();

    int read_accel(float accel[3]);
    int read_gyro(float gyro[3]);
    int read_euler(float euler[3]);
    int read_mag(float mag[3]);
    int read_temp(float *temp);

private:
    I2CInfo i2c_info_;
    uint8_t opr_mode_;
    uint8_t pwr_mode_;
    uint8_t axis_map_config_;
    uint8_t axis_map_sign_;
    uint8_t accel_g_range_;

};

#endif
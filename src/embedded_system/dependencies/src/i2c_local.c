#include "i2c_local.h"

int i2c_populate_data(struct i2c_rdwr_ioctl_data *idata, I2CInfo *info, struct i2c_msg *msgs, uint8_t read_or_write, uint8_t reg, uint8_t size, uint8_t *data) {
    if (read_or_write != I2C_M_RD && read_or_write != 0) {
        perror("Invalid I2C transaction requested");
        return -1;
    }

    // for the register selection transaction
    msgs[0].addr = info->address;
    msgs[0].flags = 0;
    msgs[0].len = 1;
    msgs[0].buf = &reg;

    // for the data exchange transaction
    msgs[1].addr = info->address;
    msgs[1].flags = read_or_write; // determines transaction type
    msgs[1].len = size;
    msgs[1].buf = data; // on read, data will be stored starting where data points

    (*idata).msgs = msgs;
    (*idata).nmsgs = 2;

    return 0;
}

int i2c_init(I2CInfo *info, const char *bus_path, uint8_t bus_num) {

    sprintf((*info).bus, "%s%d", bus_path, bus_num);
    // open the I2C bus
    (*info).fd = open((*info).bus, O_RDWR);

    if (info->fd < 0) {
        printf("I2C: failed to open bus: %s\n", info->bus);
        return -1;
    }

    return 0;

}

int i2c_read(I2CInfo *info, uint8_t reg, uint8_t size, uint8_t *data) {

    // prepare the data structures for the ioctl call
    struct i2c_rdwr_ioctl_data idata;
    struct i2c_msg msgs[2];
    
    // populate the data structures
    if (i2c_populate_data(&idata, info, msgs, I2C_M_RD, reg, size, data)) {
        return -1;
    }

    // verify that the second message is marked as a read transaction
    if (idata.msgs[1].flags != I2C_M_RD) {
        perror("I2C: not marked as read transaction");
        return -2;
    }

    // perform the I2C transaction
    if (ioctl(info->fd, I2C_RDWR, idata) != idata.nmsgs) {
        perror("I2C: communication error");
        return -3;
    }

    // data is written to where data points
    return 0;
}

int i2c_write(I2CInfo *info, uint8_t reg, uint8_t size, uint8_t *data) {

    // prepare the data structures for the ioctl call
    struct i2c_rdwr_ioctl_data idata;
    struct i2c_msg msgs[2];

    // populate the data structures
    if (i2c_populate_data(&idata, info, msgs, 0, reg, size, data)) {
        return -1;
    }

    // verify that the second message is marked as a write transaction
    if (idata.msgs[1].flags != 0) {
        perror("I2C: not marked as write transaction");
        return -2;
    }

    // perform the I2C transaction
    if (ioctl(info->fd, I2C_RDWR, idata) != idata.nmsgs) {
        perror("I2C: communication error");
        return -3;
    }

    return 0;
}


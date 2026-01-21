#include "i2c-local.h"


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

int i2c_read(I2CInfo *info, uint8_t reg, uint8_t *data) {

    if (info == NULL || data == NULL) {
        return -1;
    }

    uint8_t outbuf[1] = {reg};
    struct i2c_msg msgs[2];
    struct i2c_rdwr_ioctl_data msgset[1];

    msgs[0].addr = info->address;
    msgs[0].flags = 0;
    msgs[0].len = 1;
    msgs[0].buf = outbuf;

    msgs[1].addr = info->address;
    msgs[1].flags = I2C_M_RD;
    msgs[1].len = 1;
    msgs[1].buf = data;

    msgset[0].msgs = msgs;
    msgset[0].nmsgs = 2;

    if (ioctl(info->fd, I2C_RDWR, &msgset) < 0) {
        perror("I2C: read error");
        return -1;
    }

    return 0;
}

int i2c_write(I2CInfo *info, uint8_t reg, uint8_t *data) {

    if (info == NULL || data == NULL) {
        return -1;
    }

    uint8_t outbuf[2];
    outbuf[0] = reg;
    outbuf[1] = *data;

    struct i2c_msg msgs[1];
    struct i2c_rdwr_ioctl_data msgset[1];

    msgs[0].addr = info->address;
    msgs[0].flags = 0;
    msgs[0].len = 2;
    msgs[0].buf = outbuf;

    msgset[0].msgs = msgs;
    msgset[0].nmsgs = 1;

    if (ioctl(info->fd, I2C_RDWR, &msgset) < 0) {
        perror("I2C: write error");
        return -1;
    }

    return 0;
}

// #IMPORTANT: check if device has auto-increment functionality, if so, multi-byte reads and writes are ok
// if not, then a loop using single-byte reads and writes is needed

int i2c_multi_read(I2CInfo *info, uint8_t reg, uint8_t size, uint8_t *data) {

    if (info == NULL || data == NULL) {
        return -1;
    }

    if (size > I2C_MAX_SIZE) {
        perror("I2C: too large to read");
        return -1;
    }

    uint8_t outbuf[1] = {reg};
    struct i2c_msg msgs[2];
    struct i2c_rdwr_ioctl_data msgset[1];

    msgs[0].addr = info->address;
    msgs[0].flags = 0;
    msgs[0].len = 1;
    msgs[0].buf = outbuf;

    msgs[1].addr = info->address;
    msgs[1].flags = I2C_M_RD;
    msgs[1].len = size;
    msgs[1].buf = data;

    msgset[0].msgs = msgs;
    msgset[0].nmsgs = 2;

    if (ioctl(info->fd, I2C_RDWR, &msgset) < 0) {
        perror("I2C: communication error");
        return -1;
    }

    return 0;
}

int i2c_multi_write(I2CInfo *info, uint8_t reg, uint8_t size, uint8_t *data) {

    if (info == NULL || data == NULL) {
        return -1;
    }

    if (size > I2C_MAX_SIZE) {
        perror("I2C: too large to write");
        return -1;
    }

    uint8_t outbuf[I2C_MAX_SIZE];
    outbuf[0] = reg;
    memcpy(outbuf + sizeof(uint8_t), data, size);

    struct i2c_msg msgs[1];
    struct i2c_rdwr_ioctl_data msgset[1];

    msgs[0].addr = info->address;
    msgs[0].flags = 0;
    msgs[0].len = size + 1;
    msgs[0].buf = outbuf;

    msgset[0].msgs = msgs;
    msgset[0].nmsgs = 1;

    if (ioctl(info->fd, I2C_RDWR, &msgset) < 0) {
        perror("I2C: communication error");
        return -1;
    }

    return 0;
}


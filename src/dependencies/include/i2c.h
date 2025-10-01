#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <errno.h>
#include <string.h>

// stores permanent items related to the device: bus number and address
typedef struct _I2CInfo {
    uint8_t bus_num;
    char bus[24];
    uint8_t address;
    int fd;

} I2CInfo;

// function prototypes
// do not use i2c_populate_data() directly, it is called by i2c_read() and i2c_write()
int i2c_populate_data(struct i2c_rdwr_ioctl_data *idata, I2CInfo *info, struct i2c_msg *msgs, uint8_t read_or_write, uint8_t reg, uint8_t size, uint8_t *data);

// i2c_init() must be called before i2c_read() or i2c_write(), to set up the bus and open the file descriptor
int i2c_init(I2CInfo *info, const char *bus_path, uint8_t bus_num);
// read 'size' bytes from 'reg' into the memory pointed to by 'data'
int i2c_read(I2CInfo *info, uint8_t reg, uint8_t size, uint8_t *data);
// write 'size' bytes from the memory pointed to by 'data' into 'reg'
int i2c_write(I2CInfo *info, uint8_t reg, uint8_t size, uint8_t *data);
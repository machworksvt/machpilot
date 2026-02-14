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

#define I2C_FILE_PATH "/dev/i2c-" // default I2C file path prefix

#define I2C_MAX_SIZE 32

// stores permanent items related to the device: bus number and address
typedef struct _I2CInfo {
    uint8_t bus_num;
    char bus[24];
    uint8_t address;
    int fd;

} I2CInfo;

// function prototypes

// i2c_init() must be called before i2c_read() or i2c_write(), to set up the bus and open the file descriptor
int i2c_init(I2CInfo *info, const char *bus_path, uint8_t bus_num);
// I2C generic read command, don't use for data transfer
int i2c_read_cmd(I2CInfo *info, uint8_t *data, uint16_t size);
// read 'size' bytes from 'reg' into the memory pointed to by 'data'
int i2c_read(I2CInfo *info, uint8_t reg, uint16_t size, uint8_t *data);
// I2C generic write command, don't use for data transfer
int i2c_write_cmd(I2CInfo *info, uint8_t *data, uint16_t size);
// write 'size' bytes from the memory pointed to by 'data' into 'reg'
int i2c_write(I2CInfo *info, uint8_t reg, uint16_t size, uint8_t *data);
// deinit i2c
int i2c_deinit(I2CInfo *info);
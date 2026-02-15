#pragma once

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <stdint.h>

#define UART_FILE_PATH "/dev/ttyS"

#define EINTR_RETRIES 5

typedef struct _UARTInfo {
    char device[24];
    int fd;
    speed_t baud_rate;
    struct termios settings;
} UARTInfo;

// opens the UART device at the specified path
int uart_open(UARTInfo *info, const char *device_path);
// closes the UART device with the specified file descriptor
int uart_close(UARTInfo *info);
// configures the UART device with the specified file descriptor and baud rate
int uart_configure(UARTInfo *info, int baud_rate, int parity, int stop_bits, int data_bits, int min_chars, int timeout);
// writes data to the UART device with the specified file descriptor
ssize_t uart_write(UARTInfo *info, const uint8_t *data, size_t size);
// reads data from the UART device with the specified file descriptor
ssize_t uart_read(UARTInfo *info, uint8_t *data, size_t size);




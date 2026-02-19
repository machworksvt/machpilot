#include <uart.h>

int uart_open(UARTInfo *info, const char *device_path) {
    // open the UART file descriptor
    (*info).fd = open(device_path, O_RDWR | O_NOCTTY | O_NONBLOCK);

    // check if the UART device was opened successfully
    if (info->fd == -1) {
        perror("UART: Unable to open UART");
        return -1;
    }

    // add the device path to the UARTInfo struct
    snprintf(info->device, sizeof(info->device), "%s", device_path);

    return 0;
}
    
int uart_close(UARTInfo *info) {
    // close the UART device
    if (close((*info).fd) == -1) {
        perror("UART: Unable to close UART");
        return -1;
    }

    // reset the file descriptor
    (*info).fd = -1;

    return 0;
}

int uart_configure(UARTInfo *info, int baud_rate, int parity, int stop_bits, int data_bits, int min_chars, int timeout) {
    struct termios options; // Create the options structure

    // Get the current options for the port, should be the same as saved in info->settings
    if (tcgetattr(info->fd, &options) == -1) {
        perror("UART: Unable to get UART attributes");
        return -1;
    }

    cfmakeraw(&options); // Set raw mode

    // Set baud rate
    speed_t speed;
    switch (baud_rate) {
        case 0: speed = B0; break;
        case 50: speed = B50; break;
        case 75: speed = B75; break;
        case 110: speed = B110; break;
        case 134: speed = B134; break;
        case 150: speed = B150; break;
        case 200: speed = B200; break;
        case 300: speed = B300; break;
        case 600: speed = B600; break;
        case 1200: speed = B1200; break;
        case 1800: speed = B1800; break;
        case 2400: speed = B2400; break;
        case 4800: speed = B4800; break;
        case 9600: speed = B9600; break;
        case 19200: speed = B19200; break;
        case 38400: speed = B38400; break;
        case 57600: speed = B57600; break;
        case 115200: speed = B115200; break;
        case 230400: speed = B230400; break;
        default:
            fprintf(stderr, "UART: Unsupported baud rate %d\n", baud_rate);
            return -2;
    }

    // Set input and output baud rates
    cfsetispeed(&options, speed);
    cfsetospeed(&options, speed);

    (*info).baud_rate = speed;

    // Configure other settings
    options.c_cflag |= (CLOCAL | CREAD); // enable receiver, ignore modem control lines
    options.c_cflag &= ~CSIZE;           // clear current char size mask
    
    // parity: 0 = none, 1 = odd, 2 = even
    if (parity == 0) {
        options.c_cflag &= ~PARENB;
    } else {
        options.c_cflag |= PARENB;
        if (parity == 1)
            options.c_cflag |= PARODD;
        else
            options.c_cflag &= ~PARODD;
    }

    // stop_bits: 1 or 2
    if (stop_bits == 1) options.c_cflag &= ~CSTOPB;
    else options.c_cflag |= CSTOPB;

    // data_bits: 5..8 -> CS5..CS8
    switch (data_bits) {
    case 5: options.c_cflag |= CS5; break;
    case 6: options.c_cflag |= CS6; break;
    case 7: options.c_cflag |= CS7; break;
    case 8: options.c_cflag |= CS8; break;
    default: return -2; // unsupported data bits
    }

    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // Raw input mode
    options.c_iflag &= ~(IXON | IXOFF | IXANY);         // Disable software flow control
    options.c_oflag &= ~OPOST;                          // Raw output mode

    options.c_cc[VMIN] = min_chars;   // Minimum number of characters to read
    options.c_cc[VTIME] = timeout;    // Timeout in deciseconds (1 second)

    // Set the new options for the port
    if (tcsetattr((*info).fd, TCSANOW, &options) == -1) {
        close((*info).fd);
        (*info).fd = -1;
        perror("UART: Unable to set UART attributes");
        return -3;
    }

    // Save the settings in the UARTInfo struct
    (*info).settings = options;

    return 0;
}

ssize_t uart_write(UARTInfo *info, const uint8_t *data, size_t size) {
    // write data to the UART device
    ssize_t bytes_written = write(info->fd, data, size);
    if (bytes_written == -1) {

        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            // Unable to write right now, not an error in non-blocking mode
            return 0;
        }

        perror("UART: Write failed");

        return -1;
    }

    tcdrain(info->fd); // flush the output buffer

    return bytes_written;
}

ssize_t uart_read(UARTInfo *info, uint8_t *data, size_t size) {

    // read data from the UART device
    ssize_t bytes_read = read(info->fd, data, size);
    if (bytes_read == -1) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            // No data available right now, not an error in non-blocking mode
            return 0;
        }

        if (errno == EINTR) {
            // Interrupted by signal, retry read
            uint8_t retries = EINTR_RETRIES;
            while (retries-- > 0) {
                bytes_read = read(info->fd, data, size);
                if (bytes_read != -1) break;
                if (errno != EINTR) break; // exit if error is not EINTR
            }
            if (bytes_read != -1) return bytes_read;

            if (errno == EAGAIN || errno == EWOULDBLOCK) return 0;

            if (errno == EINTR) {
                // Still interrupted after retries
                perror("UART: Read interrupted");
                return -1;
            }
        }
        
        perror("UART: Read failed");

        return -1;
    }

    return bytes_read;
}
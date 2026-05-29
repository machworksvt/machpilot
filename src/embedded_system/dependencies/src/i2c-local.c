#include "i2c-local.h"

/**
 * ------------------------------------------------------------------------------------------------
 * NON-REALTIME
 * ------------------------------------------------------------------------------------------------
 * This block is designated as non-realtime, and does not need to be so
 * 
**/
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

/**
 * ------------------------------------------------------------------------------------------------
 * NON-REALTIME
 * ------------------------------------------------------------------------------------------------
 * This block is designated as non-realtime, and does not need to be so
 * 
**/
int i2c_deinit(I2CInfo *info) {

    if (close((*info).fd) < 0) {
        printf("I2C: failed to close bus: %s\n", info->bus);

        if (errno == EINTR) {
            printf("I2C: undetermined state while closing, freeing fd anyway: %s\n", info->bus);
            (*info).fd = 0;
        }

        return -1;
    }

    (*info).fd = 0;
    
    return 0;
}

/**
 * ------------------------------------------------------------------------------------------------
 * REALTIME
 * ------------------------------------------------------------------------------------------------
 * This block is designated as realtime, and has been reviewed.
 * Realtime practices have been confirmed and standards are adehered to.
**/
int i2c_read_cmd(I2CInfo *info, uint8_t *data, uint16_t size)
{
    int rc = 0;
    if (info == NULL) rc = -1;

    struct i2c_rdwr_ioctl_data i2c_rdwr_data;
    struct i2c_msg msgs[1];
    
    // clear ioctl data
    memset(&i2c_rdwr_data, 0, sizeof(struct i2c_rdwr_ioctl_data));

    // clear msgs data 
    memset(msgs, 0, sizeof(struct i2c_msg) * 1);
    
    // set the param 
    // see C++ Standard sec 5.16 for ternary short-circuit.
    // the following line prevents address being set to a non-existent value
    msgs[0].addr = rc == 0 ? info->address : 0x00; 
    msgs[0].flags = I2C_M_RD;
    msgs[0].buf = data;
    msgs[0].len = size;
    i2c_rdwr_data.msgs = msgs;
    i2c_rdwr_data.nmsgs = 1;
    
    // transmit, short circuits if rc is already -1
    if (rc == -1 || ioctl(info->fd, I2C_RDWR, &i2c_rdwr_data) < 0)
    {
        perror("I2C: read failed.\n");
        rc = -1;
    }
     
    return rc;
}

/**
 * ------------------------------------------------------------------------------------------------
 * REALTIME
 * ------------------------------------------------------------------------------------------------
 * This block is designated as realtime, and has been reviewed.
 * Realtime practices have been confirmed and standards are adehered to.
**/
int i2c_read(I2CInfo *info, uint8_t reg, uint16_t size, uint8_t *data) {

    int rc = 0;
    if (info == NULL || data == NULL) rc = -1;

    struct i2c_msg msgs[2];
    struct i2c_rdwr_ioctl_data msgset;

    /* clear ioctl data */
    memset(&msgset, 0, sizeof(struct i2c_rdwr_ioctl_data));
    
    /* clear msgs data */
    memset(msgs, 0, sizeof(struct i2c_msg) * 2);

    // see C++ Standard sec 5.16 for ternary short-circuit.
    // the following line prevents address being set to a non-existent value
    msgs[0].addr = rc == 0 ? info->address : 0x00; 
    msgs[0].flags = 0;
    msgs[0].len = 1;
    msgs[0].buf = &reg;

    // the following line prevents address being set to a non-existent value
    msgs[1].addr = rc == 0 ? info->address : 0x00; 
    msgs[1].flags = I2C_M_RD;
    msgs[1].len = size;
    msgs[1].buf = data;

    msgset.msgs = msgs;
    msgset.nmsgs = 2;

   // transmit, short circuits if rc is already -1
    if (rc == -1 || ioctl(info->fd, I2C_RDWR, &msgset) < 0)
    {
        perror("I2C: read failed.\n");
        rc = -1;
    }

    return rc;
}

/**
 * ------------------------------------------------------------------------------------------------
 * REALTIME
 * ------------------------------------------------------------------------------------------------
 * This block is designated as realtime, and has been reviewed.
 * Realtime practices have been confirmed and standards are adehered to.
**/
int i2c_write_cmd(I2CInfo *info, uint8_t *data, uint16_t size)
{
    int rc = 0;
    if (info == NULL) rc = -1;

    struct i2c_rdwr_ioctl_data i2c_rdwr_data;
    struct i2c_msg msgs[1];
    
    /* clear ioctl data */
    memset(&i2c_rdwr_data, 0, sizeof(struct i2c_rdwr_ioctl_data));
    
    /* clear msgs data */
    memset(msgs, 0, sizeof(struct i2c_msg) * 1);
    
    /* set the param */
    msgs[0].addr = rc == 0 ? info->address : 0x00;
    msgs[0].flags = 0;
    msgs[0].buf = data;
    msgs[0].len = size;
    i2c_rdwr_data.msgs = msgs;
    i2c_rdwr_data.nmsgs = 1;
    
    /* transmit */
    if (rc == -1 || ioctl(info->fd, I2C_RDWR, &i2c_rdwr_data) < 0)
    {
        perror("iic: write failed.\n");
        
        return -1;
    }
     
    return 0;
}

/**
 * ------------------------------------------------------------------------------------------------
 * REALTIME
 * ------------------------------------------------------------------------------------------------
 * This block is designated as realtime, and has been reviewed.
 * Realtime practices have been confirmed and standards are adehered to.
**/
int i2c_write(I2CInfo *info, uint8_t reg, uint16_t size, uint8_t *data) {

    int rc = 0;
    if (info == NULL || data == NULL) rc = -1;

    struct i2c_msg msgs[1];
    struct i2c_rdwr_ioctl_data msgset;
    uint8_t buf[size + 1];

    /* clear ioctl data */
    memset(&msgset, 0, sizeof(struct i2c_rdwr_ioctl_data));
    
    /* clear msgs data */
    memset(msgs, 0, sizeof(struct i2c_msg) * 1);
    
    /* clear sent buf */
    memset(buf, 0, sizeof(uint8_t) * (size + 1));
    buf[0] = reg;
    memcpy(&buf[1], data, size);

    msgs[0].addr = rc == 0 ? info->address : 0x00;
    msgs[0].flags = 0;
    msgs[0].len = size + 1;
    msgs[0].buf = buf;

    msgset.msgs = msgs;
    msgset.nmsgs = 1;

    if (rc == -1 || ioctl(info->fd, I2C_RDWR, &msgset) < 0)
    {
        perror("I2C: write error");
        rc = -1;
    }

    return rc;
}


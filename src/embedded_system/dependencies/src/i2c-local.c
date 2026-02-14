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

int i2c_read_cmd(I2CInfo *info, uint8_t *data, uint16_t size)
{
    if (info == NULL) return -1;

    struct i2c_rdwr_ioctl_data i2c_rdwr_data;
    struct i2c_msg msgs[1];
    
    /* clear ioctl data */
    memset(&i2c_rdwr_data, 0, sizeof(struct i2c_rdwr_ioctl_data));

    /* clear msgs data */
    memset(msgs, 0, sizeof(struct i2c_msg) * 1);
    
    /* set the param */
    msgs[0].addr = info->address;
    msgs[0].flags = I2C_M_RD;
    msgs[0].buf = data;
    msgs[0].len = size;
    i2c_rdwr_data.msgs = msgs;
    i2c_rdwr_data.nmsgs = 1;
    
    /* transmit */
    if (ioctl(info->fd, I2C_RDWR, &i2c_rdwr_data) < 0)
    {
        perror("I2C: read failed.\n");
        
        return -1;
    }
     
    return 0;
}

int i2c_read(I2CInfo *info, uint8_t reg, uint16_t size, uint8_t *data) {

    if (info == NULL || data == NULL) return -1;

    struct i2c_msg msgs[2];
    struct i2c_rdwr_ioctl_data msgset;

    /* clear ioctl data */
    memset(&msgset, 0, sizeof(struct i2c_rdwr_ioctl_data));
    
    /* clear msgs data */
    memset(msgs, 0, sizeof(struct i2c_msg) * 2);

    msgs[0].addr = info->address;
    msgs[0].flags = 0;
    msgs[0].len = 1;
    msgs[0].buf = &reg;

    msgs[1].addr = info->address;
    msgs[1].flags = I2C_M_RD;
    msgs[1].len = size;
    msgs[1].buf = data;

    msgset.msgs = msgs;
    msgset.nmsgs = 2;

    if (ioctl(info->fd, I2C_RDWR, &msgset) < 0) {
        perror("I2C: read error\n");
        return -1;
    }

    return 0;
}

int i2c_write_cmd(I2CInfo *info, uint8_t *data, uint16_t size)
{
    if (info == NULL) return -1;

    struct i2c_rdwr_ioctl_data i2c_rdwr_data;
    struct i2c_msg msgs[1];
    
    /* clear ioctl data */
    memset(&i2c_rdwr_data, 0, sizeof(struct i2c_rdwr_ioctl_data));
    
    /* clear msgs data */
    memset(msgs, 0, sizeof(struct i2c_msg) * 1);
    
    /* set the param */
    msgs[0].addr = info->address;
    msgs[0].flags = 0;
    msgs[0].buf = data;
    msgs[0].len = size;
    i2c_rdwr_data.msgs = msgs;
    i2c_rdwr_data.nmsgs = 1;
    
    /* transmit */
    if (ioctl(info->fd, I2C_RDWR, &i2c_rdwr_data) < 0)
    {
        perror("iic: write failed.\n");
        
        return -1;
    }
     
    return 0;
}

int i2c_write(I2CInfo *info, uint8_t reg, uint16_t size, uint8_t *data) {

    if (info == NULL || data == NULL) return -1;

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

    msgs[0].addr = info->address;
    msgs[0].flags = 0;
    msgs[0].len = size + 1;
    msgs[0].buf = buf;

    msgset.msgs = msgs;
    msgset.nmsgs = 1;

    if (ioctl(info->fd, I2C_RDWR, &msgset) < 0) {
        perror("I2C: write error");
        return -1;
    }

    return 0;
}


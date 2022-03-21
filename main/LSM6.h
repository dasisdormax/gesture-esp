#ifndef LSM6_h
#define LSM6_h

#include "I2CDevice.h"
#include "inc/lsm6dso_reg.h"

// LSM6DSO accelerometer and gyroscope

struct lsm6_fifo_data {
    lsm6dso_fifo_data_out_tag_t tag = { };
    int16_t x;
    int16_t y;
    int16_t z;
};

struct lsm6_fifo_bytes {
    lsm6dso_fifo_data_out_tag_t tag;
    uint8_t xl;
    uint8_t xh;
    uint8_t yl;
    uint8_t yh;
    uint8_t zl;
    uint8_t zh;
};

class LSM6 : private I2CDevice {
    public:
        LSM6(i2c_port_t port, byte addr);
        bool test();
        bool enable();
        bool fifo_flush();
        int fifo_size();
        bool read_fifo_entry(lsm6_fifo_data *data);
    private:
};

#endif
#ifndef LIS2_h
#define LIS2_h

#include "I2CDevice.h"
#include "inc/lis2mdl_reg.h"

// LIS2MDL magnetometer

struct lis2_data {
    int16_t x;
    int16_t y;
    int16_t z;
};

class LIS2 : private I2CDevice {
    public:
        LIS2(i2c_port_t port, byte addr);
        bool test();
        bool enable();
        bool read_data(lis2_data *data);
        void print_data();
    private:
};

#endif
#ifndef I2CDEVICE_H
#define I2CDEVICE_H

#include "common.h"
#include "driver/i2c.h"

class I2CDevice {
    public:
        I2CDevice(i2c_port_t port, byte addr);
        ~I2CDevice();
        bool send_byte(byte subaddr, byte data);
        bool send_bytes(byte subaddr, byte data[], byte num_bytes);
        bool read_byte(byte subaddr, byte *data);
        bool read_bytes(byte subaddr, byte data[], byte num_bytes);
        bool read_byte_plain(byte *data);

    protected:
        byte read_addr();
        byte write_addr();
        i2c_port_t port;
        byte addr;
};

#endif
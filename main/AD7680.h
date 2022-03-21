#ifndef AD7680_h_
#define AD7680_h_

#include "common.h"
#include "driver/spi_master.h"

class AD7680 {
    public:
        AD7680(spi_device_handle_t *spidev);
        bool powerOn();
        bool powerOff();
        bool read(uint16_t *data);
        bool read_averaging(uint16_t *data, int iterations);
    private:
        spi_device_handle_t *spidev;
        bool read_internal(spi_transaction_t *trans, uint16_t *data);
};

#endif
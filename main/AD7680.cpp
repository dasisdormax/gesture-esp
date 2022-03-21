#include "AD7680.h"

static const char * TAG = "AD7680";

AD7680::AD7680(spi_device_handle_t *spidev) {
    this->spidev = spidev;
}

bool AD7680::powerOn() {
    // initiate a regular read transaction, but discard the result
    uint16_t tmp;
    return read(&tmp);
}

bool AD7680::powerOff() {
    // For now, do nothing as the power-off / power-on sequence caused issues previously
    return true;
    // when the transaction length is less than 10, the AD7680
    // will enter power off mode.
    spi_transaction_t trans = {
        .flags = SPI_TRANS_USE_RXDATA,
        .length = 4,
    };
    spi_device_t *dev = *spidev;
    spi_device_acquire_bus(dev, portMAX_DELAY);
    spi_device_polling_transmit(dev, &trans);
    spi_device_release_bus(dev);
    return true;
}

bool AD7680::read_averaging(uint16_t *data, int iterations) {
    spi_transaction_t trans = {
        .flags = SPI_TRANS_USE_RXDATA,
        .length = 24,
    };
    spi_device_t *dev = *spidev;
    spi_device_acquire_bus(dev, portMAX_DELAY);
    uint32_t sum = 0;
    for(int i = 0; i < iterations; i++) {
        if(!read_internal(&trans, data)) {
            spi_device_release_bus(dev);
            return false;
        }
        sum += *data;
    }
    *data = sum / iterations;
    spi_device_release_bus(dev);
    return true;
}

bool AD7680::read_internal(spi_transaction_t *trans, uint16_t *data) {
    spi_device_t *dev = *spidev;
    spi_device_polling_transmit(dev, trans);
    *data = (((uint16_t)trans->rx_data[0]) << 12) + (((uint16_t)trans->rx_data[1]) << 4) + (trans->rx_data[2] >> 4);
    return true;
}

bool AD7680::read(uint16_t *data) {
    return read_averaging(data, 1);
}
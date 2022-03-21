#include "I2CDevice.h"

I2CDevice::I2CDevice(i2c_port_t port, byte addr) {
    this->port = port;
    this->addr = addr;
}

I2CDevice::~I2CDevice() {}

byte I2CDevice::read_addr() {
    return addr << 1 | 1;
}

byte I2CDevice::write_addr() {
    return addr << 1;
}

bool I2CDevice::send_byte(byte subaddr, byte data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, write_addr(), 1);
    i2c_master_write_byte(cmd, subaddr, 1);
    i2c_master_write_byte(cmd, data, 1);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(port, cmd, 10);
    i2c_cmd_link_delete(cmd);
    return err == ESP_OK;
}

bool I2CDevice::send_bytes(byte subaddr, byte data[], byte num_bytes) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, write_addr(), 1);
    i2c_master_write_byte(cmd, subaddr, 1);
    i2c_master_write(cmd, data, num_bytes, 1);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(port, cmd, 10);
    i2c_cmd_link_delete(cmd);
    return err == ESP_OK;

}

bool I2CDevice::read_byte_plain(byte *data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, read_addr(), 1);
    i2c_master_read_byte(cmd, data, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(port, cmd, 10);
    i2c_cmd_link_delete(cmd);
    return err == ESP_OK;
}

bool I2CDevice::read_byte(byte subaddr, byte *data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, write_addr(), 1);
    i2c_master_write_byte(cmd, subaddr, 1);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, read_addr(), 1);
    i2c_master_read_byte(cmd, data, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(port, cmd, 10);
    i2c_cmd_link_delete(cmd);
    return err == ESP_OK;
}

bool I2CDevice::read_bytes(byte subaddr, byte data[], byte num_bytes) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, write_addr(), 1);
    i2c_master_write_byte(cmd, subaddr, 1);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, read_addr(), 1);
    i2c_master_read(cmd, data, num_bytes, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(port, cmd, 10);
    i2c_cmd_link_delete(cmd);
    return err == ESP_OK;
}
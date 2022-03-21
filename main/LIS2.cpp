#include "LIS2.h"

static const char * TAG = "LIS2";

LIS2::LIS2(i2c_port_t port, byte addr) : I2CDevice(port, addr) {

}

bool LIS2::test() {
    byte data;
    read_byte(LIS2MDL_WHO_AM_I, &data);
    ESP_LOGI(TAG, "WHO_AM_I is %x, should be 40", data);
    return data == 0x40;
}

bool LIS2::enable() {
    lis2mdl_cfg_reg_a_t rega;
    read_byte(LIS2MDL_CFG_REG_A, (byte *)&rega);
    rega.comp_temp_en = 1;
    rega.lp = LIS2MDL_LOW_POWER;
    rega.odr = LIS2MDL_ODR_20Hz;
    rega.md = LIS2MDL_CONTINUOUS_MODE;
    send_byte(LIS2MDL_CFG_REG_A, *(byte *)&rega);

    lis2mdl_cfg_reg_b_t regb;
    read_byte(LIS2MDL_CFG_REG_B, (byte *)&regb);
    regb.lpf = 1;
    send_byte(LIS2MDL_CFG_REG_B, *(byte *)&regb);

    lis2mdl_cfg_reg_c_t regc;
    read_byte(LIS2MDL_CFG_REG_C, (byte *)&regc);
    regc.bdu = 1;
    send_byte(LIS2MDL_CFG_REG_C, *(byte *)&regc);
    
    return true;
}

void LIS2::print_data() {
    lis2_data data;
    read_data(&data);
    ESP_LOGI(TAG, "MM: %06d,%06d,%06d", data.x, data.y, data.z);
}

bool LIS2::read_data(lis2_data *data) {
    return read_bytes(LIS2MDL_OUTX_L_REG, (byte *)data, 6);
}
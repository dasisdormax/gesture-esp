#include "LSM6.h"

static const char * TAG = "LSM6";

LSM6::LSM6(i2c_port_t port, byte addr) : I2CDevice(port, addr) {

}

bool LSM6::test() {
    byte data;
    read_byte(LSM6DSO_WHO_AM_I, &data);
    ESP_LOGI(TAG, "WHO_AM_I is %x, should be 6C", data);
    return data == 0x6C;
}

bool LSM6::enable() {
    lsm6dso_ctrl1_xl_t xreg;
    read_byte(LSM6DSO_CTRL1_XL, (byte *)&xreg);
    xreg.odr_xl = LSM6DSO_XL_ODR_104Hz;
    xreg.fs_xl = LSM6DSO_4g;
    send_byte(LSM6DSO_CTRL1_XL, *(byte *)&xreg);

    lsm6dso_ctrl2_g_t greg;
    read_byte(LSM6DSO_CTRL2_G, (byte *)&greg);
    greg.odr_g = LSM6DSO_GY_ODR_104Hz;
    greg.fs_g = LSM6DSO_1000dps;
    send_byte(LSM6DSO_CTRL2_G, *(byte *)&greg);

    lsm6dso_fifo_ctrl3_t freg3;
    read_byte(LSM6DSO_FIFO_CTRL3, (byte *)&freg3);
    freg3.bdr_xl = LSM6DSO_XL_BATCHED_AT_104Hz;
    freg3.bdr_gy = LSM6DSO_GY_BATCHED_AT_104Hz;
    send_byte(LSM6DSO_FIFO_CTRL3, *(byte *)&freg3);

    lsm6dso_fifo_ctrl4_t freg4;
    read_byte(LSM6DSO_FIFO_CTRL4, (byte *)&freg4);
    freg4.fifo_mode = LSM6DSO_STREAM_MODE;
    send_byte(LSM6DSO_FIFO_CTRL4, *(byte *)&freg4);

    fifo_flush();
    
    return true;
}

int LSM6::fifo_size() {
    byte fifo_status[2];
    read_bytes(LSM6DSO_FIFO_STATUS1, fifo_status, 2);
    return fifo_status[0] + ((int)(fifo_status[1] & 0b11) << 8);
}

bool LSM6::fifo_flush() {
    lsm6_fifo_data data = {};
    int size = fifo_size();
    for(int i = 0; i < size; i++) {
        read_fifo_entry(&data);
    }
    ESP_LOGI(TAG, "flushed %d entries from FIFO", size);
    return true;
}

bool LSM6::read_fifo_entry(lsm6_fifo_data *data) {
    lsm6_fifo_bytes bytes;
    if(!read_bytes(LSM6DSO_FIFO_DATA_OUT_TAG, (byte *)&bytes, 7)) return false;
    data->tag = bytes.tag;
    data->x = bytes.xl | (((int16_t)bytes.xh) << 8);
    data->y = bytes.yl | (((int16_t)bytes.yh) << 8);
    data->z = bytes.zl | (((int16_t)bytes.zh) << 8);
    return true;
}
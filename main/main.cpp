/* Blink Example
   This example code is in the Public Domain (or CC0 licensed, at your option.)
   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
//#include <stdio.h>

#include "AD5933.h"
#include "AD7680.h"
#include "LSM6.h"
#include "LIS2.h"
#include "wireless.h"
#include "common.h"

#include "Matrix3.h"



////////////////////////////////////////////////////
//////// VARIABLE DEFINITIONS
////////////////////////////////////////////////////

static const char *TAG = "main";

// Impedance measurement
#ifdef AD5933_ADDR
    AD5933 adz(I2C_PORT, AD5933_ADDR);

    long inv_gain[ARR_SIZE];
    int phase[ARR_SIZE];

    int zm_fb_values[] = ZM_FB_RES;
    int zm_cal_values[] = ZM_CAL_RES;
    int zm_cal_channels[] = ZM_CAL_CH;

    int Z_values[NUM_CHANNELS];

    int16_t tmp_re[ARR_SIZE];
    int16_t tmp_im[ARR_SIZE];
#endif

// Resistance measurement
int rm_range;

#ifdef SPI_BUS
    uint16_t tmp_r;
    int R_values[NUM_CHANNELS];

    int rm_ref_values[] = RM_REF_RES;
    int rm_cal_values[] = RM_CAL_RES;
    int rm_cal_channels[] = RM_CAL_CH;

    spi_device_handle_t spidev;
    AD7680 adr(&spidev);
#endif

// Accelerometer and gyroscope
#ifdef LSM6_ADDR
    LSM6 gyxl(I2C_PORT, LSM6_ADDR);
    Matrix3 rot;
    lsm6_fifo_data xl;
    lsm6_fifo_data gy;

    const float deg2rad = 3.14159265 / 180;
    // Measurement range, 1000 dps, measurements
    const float gyro_base = deg2rad * GYRO_SENS_DPS / 32767;
    uint64_t gyro_last_time;

    int fifo_size;

    int gyro_counter = 0;
#endif

// Magnetometer
#ifdef LIS2_ADDR
    LIS2 magn(I2C_PORT, LIS2_ADDR);
    lis2_data mag;
#endif




////////////////////////////////////////////////////
//////// GESTURE DETECTION SETUP
////////////////////////////////////////////////////

void mux_config(gpio_num_t pin) {
    if(pin == GPIO_NUM_NC) return;
    gpio_set_direction(pin, GPIO_MODE_OUTPUT);
    gpio_set_level(pin, 0);
}

// Selects the channel of the R and Z subject multiplexers.
// 0-9 -> subjects, 10-15 -> calibration resistors
void select_channel (int channel) {
    gpio_set_level(SUBJ_A0_GPIO, channel & 0b0001);
    gpio_set_level(SUBJ_A1_GPIO, channel & 0b0010);
    gpio_set_level(SUBJ_A2_GPIO, channel & 0b0100);
    gpio_set_level(SUBJ_A3_GPIO, channel & 0b1000);
}

// Selects the resistance measurement range
//   0 -> 100k - 1M   Ohm
//   1 -> 10k  - 100k Ohm
//   2 -> 1k   - 10k  Ohm
//   3 -> 100  - 1k   Ohm
void set_rm_range (int range) {
    rm_range = range;
    gpio_set_level(RANGE_A0_GPIO, range & 0b01);
    gpio_set_level(RANGE_A1_GPIO, range & 0b10);
}

// Selects the impedance amplifier feedback resistor
// The resistor values are board revision dependent
void set_zm_fb (int fb) {
    int value = 0;
    if(fb > 0 && fb < sizeof (zm_fb_values) / sizeof (int)) {
        value = zm_fb_values[fb];
    }
    if(value) {
        ESP_LOGD(TAG, "Selecting feedback resistor %d with value of %d Ohm.", fb, value);
    } else {
        ESP_LOGW(TAG, "The feedback resistor %d is unavailable for this board revision.", fb);
    }
    gpio_set_level(FB_A0_GPIO, fb & 0b01);
    gpio_set_level(FB_A1_GPIO, fb & 0b10);
}

void set_enabled (bool enabled) {
    gpio_set_level(MEASURE_EN_GPIO, enabled);
}


void gesture_setup() {
    ////// CONFIGURE INTERFACES
    // Configure I2C
    #ifdef I2C_PORT
        i2c_config_t i2ccfg = {
            .mode = I2C_MODE_MASTER,
            .sda_io_num = SDA_GPIO,
            .scl_io_num = SCL_GPIO,
            .sda_pullup_en = GPIO_PULLUP_DISABLE,
            .scl_pullup_en = GPIO_PULLUP_DISABLE
        };
        i2ccfg.master.clk_speed = I2C_FREQ;
        i2c_param_config(I2C_PORT, &i2ccfg);
        i2c_driver_install(I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);
    #endif

    // Configure SPI
    #ifdef SPI_BUS
        spi_bus_config_t spicfg = {
            .mosi_io_num = MOSI_GPIO,
            .miso_io_num = MISO_GPIO,
            .sclk_io_num = SCK_GPIO,
            .quadwp_io_num = GPIO_NUM_NC,
            .quadhd_io_num = GPIO_NUM_NC,
            .max_transfer_sz = 32,
        };
        spi_bus_initialize(SPI_BUS, &spicfg, SPI_DMA_CH_AUTO);
        spi_device_interface_config_t devcfg = {
            .mode = 3, // CPOL = 1, CPHA = 1
            .clock_speed_hz = SPI_FREQ,
            .spics_io_num = CS0_GPIO,
            .queue_size = 2,
        };
        spi_bus_add_device(SPI_BUS, &devcfg, &spidev);
    #endif

    // Configure MUX GPIOs
    mux_config(MEASURE_EN_GPIO);
    mux_config(SUBJ_A0_GPIO);
    mux_config(SUBJ_A1_GPIO);
    mux_config(SUBJ_A2_GPIO);
    mux_config(SUBJ_A3_GPIO);
    mux_config(RANGE_A0_GPIO);
    mux_config(RANGE_A1_GPIO);
    mux_config(FB_A0_GPIO);
    mux_config(FB_A1_GPIO);
    
    ////// CONFIGURE SENSORS
    //Perform initial configuration of AD5933. Fail if any one of these fail.
    #ifdef AD5933_ADDR
        if (!(adz.reset() &&
            adz.setInternalClock(true) &&
            adz.setStartFrequency(START_FREQ) &&
            adz.setIncrementFrequency(FREQ_INCR) &&
            adz.setNumberIncrements(NUM_INCR) &&
            adz.setRange(VRANGE_4) &&
            adz.setPGAGain(PGA_GAIN_X1) &&
            adz.setSettlingCycles(SETTLING_CYCLES)
        )) {
            ESP_LOGE(TAG, "FAILED initializing AD5933!");
            vTaskSuspend(NULL);
        }
    #endif

    //Configure LSM6DSO
    #ifdef LSM6_ADDR
        gyxl.test();
        gyxl.enable();
    #endif

    //Configure LIS2MDL
    #ifdef LIS2_ADDR
        magn.test();
        magn.enable();
    #endif

    // Setup Wireless connection
    wireless_init();
}




////////////////////////////////////////////////////
//////// GESTURE DETECTION LOOP
////////////////////////////////////////////////////

int loop_counter = 0;

// Result string buffer
char results[512];

#ifdef AD5933_ADDR
int calculate_impedance() {
    int ref_impedance = 0;
    long magnitude;
    int re;
    int im;
    // for now, average magnitudes over the measurements of each channel
    // update if necessary
    for (int i = 0; i < ARR_SIZE; i++) {
        re = tmp_re[i];
        im = tmp_im[i];
        magnitude = fast_int_sqrt(re * re + im * im);
        if(magnitude != 0)
            ref_impedance += inv_gain[i] * ARR_SIZE / magnitude;
    }
    return ref_impedance;
}
#endif

#ifdef SPI_BUS
void rm_measure_and_check() {
    // Power on and allow the value to settle after a range or channel change
    adr.powerOn();

    #ifndef ADC_FIXED_RANGE
        // Shorter reading to determine if the range is correct
        adr.read_averaging(&tmp_r, ADC_AVERAGING / 5 + 1);

        // Calculate the optimal range for the reading
        int new_range = rm_range;
        if(tmp_r < 13000) {
            new_range++;
            if(tmp_r < 1750) new_range++;
            if(new_range > 3) new_range = 3;
        } else if(tmp_r > 52500) {
            new_range--;
            if(tmp_r > 63800) new_range--;
            if(new_range < 0) new_range = 0;
        }
        if(new_range != rm_range) {
            // If the range should be changed, update range and check again.
            set_rm_range(new_range);
            rm_measure_and_check();
            return;
        }
    #endif
    // If the range is correct already, average multiple voltage readings.
    adr.read_averaging(&tmp_r, ADC_AVERAGING);
}

int calculate_resistance() {
    // Value V = 65535 * R / (R + R_REF)
    // R = V R_REF / (65535 - V)
    uint64_t r_ref = rm_ref_values[rm_range];
    uint16_t div = 65535 - tmp_r;
    if(div < 100) return 0;
    int result = r_ref * tmp_r / div;
    return result;
}
#endif

// Starts the next resistance and impedance measurement
void rz_start(int channel) {
    #ifdef AD5933_ADDR
        adz.startFrequencySweep();
    #endif
}

void imu_prepare() {
    // Collect magnetometer data: once per cycle
    #ifdef LIS2_ADDR
        magn.read_data(&mag);
    #endif

    // Query fifo size once per cycle, read until empty
    #ifdef LSM6_ADDR
        fifo_size = gyxl.fifo_size();
    #endif
}

void imu_read(int channel) {
    // Collect accelerometer / gyroscope data
    #ifdef LSM6_ADDR
        int rem = NUM_CHANNELS - channel;
        int fifo_num = (fifo_size / rem / 2) * 2 + 2;
        if(fifo_num > fifo_size) fifo_num = 0;
        float gyro_factor = 0;
        if(fifo_num) {
            // TODO: use timestamps provided by LSM6 instead of ESP32 timestamp
            uint64_t current_time = esp_timer_get_time();
            gyro_factor = gyro_base * (current_time - gyro_last_time) * 2 / fifo_num / 1.e6f;
            gyro_last_time = current_time;
        }
        lsm6_fifo_data data;
        float rx;
        float ry;
        float rz;
        Matrix3 rtmp;
        for(int i = 0; i < fifo_num; i++) {
            gyxl.read_fifo_entry(&data);
            // for now, ignore acceleration and only handle gyroscope
            switch(data.tag.tag_sensor) {
                case LSM6DSO_GYRO_NC_TAG: 
                    gy = data;
                    rx = data.x * gyro_factor;
                    ry = data.y * gyro_factor;
                    rz = data.z * gyro_factor;
                    rtmp.set_rot_3d(rx, ry, rz);
                    rot.mul_left(&rtmp);
                    gyro_counter ++;
                    break;
                case LSM6DSO_XL_NC_TAG:
                    xl = data;
                    break;
                default:
                    ESP_LOGI(TAG, "LSM6 FIFO Packet: [%d], %d, %d, %d", data.tag.tag_sensor, data.x, data.y, data.z);
            }
        }
        fifo_size -= fifo_num;
    #endif
}

// Finishes the current measurement and sets the pointers
// for the results to be stored
void rz_finish(int channel) {
    #ifdef SPI_BUS
        rm_measure_and_check();
        adr.powerOff();
    #endif

    #ifdef AD5933_ADDR
        // Wait until the frequency sweep is done
        adz.setDataStore(tmp_re, tmp_im);
        while(!adz.advanceFrequencySweep(FAST_UNSAFE)) ;
    #endif
}

// Calculates the results of the previously measured
// resistance and impedance channel and writes them
// to the storage
void rz_calculate_result(int channel) {
    if(channel < 0) return;

    #ifdef AD5933_ADDR
        // Process impedance results
        Z_values[channel] = calculate_impedance();
    #endif

    #ifdef SPI_BUS
        // Process resistance results
        R_values[channel] = calculate_resistance();
    #endif
}

void rz_poweron() {
    set_enabled(true);
}

// Sets the resistance and impedance circuits into standby
void rz_poweroff() {
    set_enabled(false);
    #ifdef AD5933_ADDR
        adz.powerOff();
    #endif
}

void create_result_string(int64_t timestamp) {
    int delta = esp_timer_get_time() - timestamp;
    char *ptr = results;
    ptr += sprintf(ptr, "[%7d+%2d.%dms]", (int)(timestamp / 1000), delta / 1000, (delta / 100) % 10);
    int i;
    #ifdef SPI_BUS
        ptr += sprintf(ptr, "\n  R");
        for(i = 0; i < NUM_CHANNELS; i++) {
            int R = R_values[i];
            if(R <= 0 || R > 1000000) {
                ptr += sprintf(ptr, ":-");
            } else {
                ptr += sprintf(ptr, ":%d", R);
            }
        }
    #endif
    #ifdef AD5933_ADDR
        ptr += sprintf(ptr, "\n  Z");
        for(i = 0; i < NUM_CHANNELS; i++) {
            int Z = Z_values[i];
            if(Z <= 0 || Z > 1000000) {
                ptr += sprintf(ptr, ":-");
            } else {
                ptr += sprintf(ptr, ":%d", Z);
            }
        }
    #endif
    #ifdef LSM6_ADDR
        ptr += sprintf(ptr, "\n  X:%d:%d:%d", xl.x, xl.y, xl.z);
        ptr += sprintf(ptr, "\n  G:%d:%d:%d", gy.x, gy.y, gy.z);
    #endif
    #ifdef LIS2_ADDR
        ptr += sprintf(ptr, "\n  M:%d:%d:%d", mag.x, mag.y, mag.z);
    #endif
    sprintf(ptr, "\n");
}

void print_info() {
    if(!PRINT_INTERVAL) return;
    if(PRINT_INTERVAL > 1 && loop_counter % PRINT_INTERVAL) return;
    printf("%s", results);
}

void gesture_loop ()  {
    int64_t start = esp_timer_get_time();

    // Perform measurements
    rz_poweron();
    imu_prepare();
    for(int channel = 0; channel < NUM_CHANNELS; channel++) {
        select_channel(channel);
        rz_start(channel);
        rz_calculate_result(channel - 1);
        imu_read(channel);
        rz_finish(channel);
    }
    rz_poweroff();
    rz_calculate_result(NUM_CHANNELS - 1);

    create_result_string(start);
    wireless_send(results);
    print_info();
    loop_counter++;
}




////////////////////////////////////////////////////
//////// CALIBRATION AND ENTRY POINT
////////////////////////////////////////////////////

void gesture_calibrate() {
    // Calibrate AD5933
    rz_poweron();
    
    // TODO: calibrate multiple feedback ranges
    int cal_num = 1; // 49.9 k
    int fb_num = 1;
    select_channel(zm_cal_channels[cal_num]); // 49.9 k
    set_zm_fb(fb_num);
    #ifdef AD5933_ADDR
        bool success = adz.calibrate(inv_gain, phase, zm_cal_values[cal_num], ARR_SIZE);
        if(!success) {
            ESP_LOGE(TAG, "FAILED calibrating AD5933!");
            vTaskSuspend(NULL);
        }
    #endif

    #ifdef SPI_BUS
        // TODO: through calibration, determine the actual values of the reference
        //       resistors to improve accuracy and prevent jumps when switching ranges.
        #ifdef ADC_FIXED_RANGE
            set_rm_range(ADC_FIXED_RANGE);
        #else
            set_rm_range(1);
        #endif

        // This is just a check to see if the values are about as expected
        int cal_ch = 1;
        select_channel(rm_cal_channels[cal_ch]);
        adr.powerOn();
        adr.read_averaging(&tmp_r, ADC_AVERAGING * 2);
        int r_calc = calculate_resistance();
        ESP_LOGI(TAG, "Resistance check 1: raw data: %u, expected R: %d, calculated R = %d.",
                      tmp_r, rm_cal_values[cal_ch], r_calc);

        cal_ch = 2;
        select_channel(rm_cal_channels[cal_ch]);
        adr.powerOn();
        adr.read_averaging(&tmp_r, ADC_AVERAGING * 2);
        r_calc = calculate_resistance();
        ESP_LOGI(TAG, "Resistance check 2: raw data: %u, expected R: %d, calculated R = %d.",
                      tmp_r, rm_cal_values[cal_ch], r_calc);

        adr.powerOff();
    #endif

    rz_poweroff();
}

extern "C" void app_main(void)
{
    gesture_setup();
    gesture_calibrate();
    TickType_t tick = xTaskGetTickCount();
    const int intervalTicks = LOOP_INTERVAL_MS / portTICK_PERIOD_MS;
    while (true)
    {
        gesture_loop();
        vTaskDelayUntil(&tick, intervalTicks);
    }
}


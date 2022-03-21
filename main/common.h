#ifndef helpers_h
#define helpers_h

#include <cmath>
#include <cstring>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "settings.h"

//// HELPER FUNCTIONS

typedef uint8_t byte;

long fast_int_sqrt(long n);
void digitalWriteChecked(byte pin, byte value);
void sleepMicroseconds(long sleepMicros);



//// HARDWARE INFORMATION

// Gesture-ESP hardware revision 1.3
#ifdef HW_IS_REV_1_3

//////// PIN CONFIGURATION
// General
#define LED_GPIO GPIO_NUM_2
#define BTN_GPIO GPIO_NUM_0
// I2C
#define SCL_GPIO GPIO_NUM_4
#define SDA_GPIO GPIO_NUM_15
// SPI
#define CS0_GPIO GPIO_NUM_5
#define MISO_GPIO GPIO_NUM_19
#define MOSI_GPIO GPIO_NUM_23
#define SCK_GPIO GPIO_NUM_18
// Multiplexers
#define MEASURE_EN_GPIO GPIO_NUM_25
#define SUBJ_A0_GPIO GPIO_NUM_26
#define SUBJ_A1_GPIO GPIO_NUM_27
#define SUBJ_A2_GPIO GPIO_NUM_32
#define SUBJ_A3_GPIO GPIO_NUM_33
#define RANGE_A0_GPIO GPIO_NUM_16
#define RANGE_A1_GPIO GPIO_NUM_17
#define FB_A0_GPIO GPIO_NUM_21
#define FB_A1_GPIO GPIO_NUM_22

// Device addresses (some are configurable)
#define LSM6_ADDR   (0x6A)
#define LIS2_ADDR   (0x1E)
#define AD5933_ADDR (0x0D)
#define SPI_BUS SPI3_HOST

// Define resistance values
#define ZM_FB_RES  { 0, 12000, 102000, 332000 }
#define ZM_CAL_RES { 4700, 49900, 330000 }
#define ZM_CAL_CH  { 10, 12, 14 }
#define RM_REF_RES { 330000, 33000, 3300, 330 }
#define RM_CAL_RES { 1000, 10000, 100000 }
#define RM_CAL_CH  { 10, 12, 14 }

//////// Helper constants and variables
#define I2C_PORT I2C_NUM_0
#define I2C_FREQ 300000
#define SPI_FREQ 2000000
#define NUM_CHANNELS 10

#endif

// Conservative default settings
#ifndef LED_GPIO
#define LED_GPIO GPIO_NUM_NC
#endif
#ifndef BTN_GPIO
#define BTN_GPIO GPIO_NUM_NC
#endif
#ifndef MEASURE_EN_GPIO
#define MEASURE_EN_GPIO GPIO_NUM_NC
#endif
#ifndef SUBJ_A0_GPIO
#define SUBJ_A0_GPIO GPIO_NUM_NC
#endif
#ifndef SUBJ_A1_GPIO
#define SUBJ_A1_GPIO GPIO_NUM_NC
#endif
#ifndef SUBJ_A2_GPIO
#define SUBJ_A2_GPIO GPIO_NUM_NC
#endif
#ifndef SUBJ_A3_GPIO
#define SUBJ_A3_GPIO GPIO_NUM_NC
#endif
#ifndef RANGE_A0_GPIO
#define RANGE_A0_GPIO GPIO_NUM_NC
#endif
#ifndef RANGE_A1_GPIO
#define RANGE_A1_GPIO GPIO_NUM_NC
#endif
#ifndef FB_A0_GPIO
#define FB_A0_GPIO GPIO_NUM_NC
#endif
#ifndef FB_A1_GPIO
#define FB_A1_GPIO GPIO_NUM_NC
#endif
#ifndef I2C_FREQ
#define I2C_FREQ 100000
#endif
#ifndef NUM_CHANNELS
#define NUM_CHANNELS 4
#endif


#endif

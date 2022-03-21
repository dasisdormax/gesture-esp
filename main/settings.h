#ifndef settings_h
#define settings_h

#include "inc/lsm6dso_reg.h"

//////// HARDWARE

// Set the correct hardware revision, otherwise issues or damage might occur.
// #define   HW_IS_PROTOTYPE
#define   HW_IS_REV_1_3


//////// Program settings
#define   LOOP_INTERVAL_MS 50

#define   PRINT_INTERVAL   1      // Interval of printing the data to stdout
                                  // (every n loop iterations, 0 disables printing)

#define   FAST_UNSAFE      false  // Enables speed optimizations that may decrease
                                  // measurement stability and accuracy


//////// AD5933 Impedance converter
#define   START_FREQ       5000   // Starting frequency in Hz
#define   NUM_INCR         0      // Number of incrementing steps
#define   FREQ_INCR        10     // Frequency increment for each step in Hz

#define   SETTLING_CYCLES  (START_FREQ / 2000 + 1)      // Number of settling cycles before each measurement

#define   ARR_SIZE         (NUM_INCR + 1) // Array size and total number of measurements
#define   END_FREQ         (START_FREQ + NUM_INCR * FREQ_INCR)

//////// AD7680 ADC
#undef    ADC_FIXED_RANGE         // Fixed range setting, if unset, the range is updated at each measurement
//#define   ADC_FIXED_RANGE  1
#define   ADC_AVERAGING    20     // How many readings to average

//////// LSM6DSO Accelerometer and Gyroscope
#define   GYRO_SENS_DPS    1000.0
#define   GYRO_SENS        LSM6DSO_1000dps
#define   XL_SENS_G        8.0
#define   XL_SENS          LSM6DSO_8g


#endif
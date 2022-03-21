// vim: ts=4:sw=4:sts=4:et

/**
 * @file AD5933.cpp
 * @brief Library code for AD5933
 *
 * Library code for AD5933. 
 
 */

#include "AD5933.h"

//////// INTERNAL CONSTANTS ////////

// Address pointer for specific operations
#define ADDR_PTR        (0xB0)
// Control Register
#define CTRL_REG1       (0x80)
#define CTRL_REG2       (0x81)
// Start Frequency Register
#define START_FREQ_1    (0x82)
#define START_FREQ_2    (0x83)
#define START_FREQ_3    (0x84)
// Frequency increment register
#define INC_FREQ_1      (0x85)
#define INC_FREQ_2      (0x86)
#define INC_FREQ_3      (0x87)
// Number of increments register
#define NUM_INC_1       (0x88)
#define NUM_INC_2       (0x89)
// Number of settling time cycles register
#define NUM_SCYCLES_1   (0x8A)
#define NUM_SCYCLES_2   (0x8B)
// Status register
#define STATUS_REG      (0x8F)
// Temperature data register
#define TEMP_DATA_1     (0x92)
#define TEMP_DATA_2     (0x93)
// Real data register
#define REAL_DATA_1     (0x94)
#define REAL_DATA_2     (0x95)
// Imaginary data register
#define IMAG_DATA_1     (0x96)
#define IMAG_DATA_2     (0x97)

// Block command addresses
#define BLOCK_WRITE     (0b10100000)
#define BLOCK_READ      (0b10100001)

static const char *TAG = "AD5933";

//////// IMPLEMENTATION ////////

AD5933::AD5933(i2c_port_t port, byte addr) : I2CDevice(port, addr) {
    currentControlLow = 0x00;
    currentControlHigh = 0xA0;
    currentSettlingCycles = 0;
    currentStartFrequency = 0;
    ready = false;
    lastStatus = 0;
}

bool AD5933::setAddress(byte address) {
    return sendByte(ADDR_PTR, address);
}

/**
 * Request to read multiple bytes from the AD5933.
 *
 * @param address Address of register requesting data from
 * @param value Pointer to a byte where the return value should be stored, or
 *        where the error code will be stored if fail.
 * @return Success or failure
 */
bool AD5933::getBytes(byte address, byte *values, byte numValues) {
    if(!setAddress(address)) return false;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, write_addr(), 1);
    i2c_master_write_byte(cmd, BLOCK_READ, 1);
    i2c_master_write_byte(cmd, numValues, 1);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, read_addr(), 1);
    i2c_master_read(cmd, values, numValues, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(port, cmd, 10);
    i2c_cmd_link_delete(cmd);
    return err == ESP_OK;
}

/**
 * Request to read a byte from the AD5933.
 *
 * @param address Address of register requesting data from
 * @param value Pointer to a byte where the return value should be stored, or
 *        where the error code will be stored if fail.
 * @return Success (true or false)
 */
bool AD5933::getByte(byte address, byte *value) {
    return setAddress(address) && read_byte_plain(value);
}

/**
 * Write a byte to a register on the AD5933.
 *
 * @param address The register address to write to
 * @param value The byte to write to the address
 * @return Success or failure of transmission
 */
bool AD5933::sendByte(byte address, byte value) {
    return send_byte(address, value);
}

/**
 * Write multiple bytes to registers on the AD5933.
 *
 * @param address The register address to write to
 * @param value The byte to write to the address
 * @return Success or failure of transmission
 */
bool AD5933::sendBytes(byte address, byte *values, byte numValues) {
    if (!setAddress(address)) return false;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, write_addr(), 1);
    i2c_master_write_byte(cmd, BLOCK_WRITE, 1);
    i2c_master_write_byte(cmd, numValues, 1);
    i2c_master_write(cmd, values, numValues, 1);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(port, cmd, 10);
    i2c_cmd_link_delete(cmd);
    return err == ESP_OK;
}


/**
 * Set the control mode register, CTRL_REG1. This is the register where the
 * current command needs to be written to so this is used a lot.
 *
 * @param mode The control mode to set
 * @return Success or failure
 */
bool AD5933::setControlMode(byte mode) {
    // Get the current value of the control register

    // Replace upper 4 bits with bits from control mode
    byte newCCH = (currentControlHigh & 0x0F) | (mode & 0xF0);

    // Write back to the register
    return sendByte(CTRL_REG1, newCCH);
}

/**
 * Reset the AD5933. This interrupts a sweep if one is running, but the start
 * frequency, number of increments, and frequency increment register contents
 * are not overwritten, but an initialize start frequency command is required
 * to restart a frequency sweep.
 *
 * @return Success or failure
 */
bool AD5933::reset() {
    // Get the current value of the control register
    byte newCCL = currentControlLow | CTRL_RESET;
    sweepIndex = -1;

    return sendByte(CTRL_REG2, newCCL);
}

/**
 * Set enable temperature measurement. This interferes with frequency sweep
 * operation, of course.
 *
 * @param enable Option to enable to disable temperature measurement.
 * @return Success or failure
 */
bool AD5933::enableTemperature(byte enable) {
    // If enable, set temp measure bits. If disable, reset to no operation.
    if (enable == TEMP_MEASURE) {
        return setControlMode(CTRL_TEMP_MEASURE);
    } else {
        return setControlMode(CTRL_STANDBY_MODE);
    }
}

/**
 * Get the temperature reading from the AD5933. Waits until a temperature is
 * ready. Also ensures temperature measurement mode is active.
 *
 * @return The temperature in celcius, or -1 if fail.
 */
double AD5933::getTemperature() {
    // Set temperature mode
    if (enableTemperature(TEMP_MEASURE)) {
        // Wait for a valid temperature to be ready
        waitForStatusFlag(STATUS_TEMP_VALID);

        // Read raw temperature from temperature registers
        byte rawTemp[2];
        if (getBytes(TEMP_DATA_1, rawTemp, 2))
        {
            // Combine raw temperature bytes into an interger. The ADC
            // returns a 14-bit 2's C value where the 14th bit is a sign
            // bit. As such, we only need to keep the bottom 13 bits.
            int rawTempVal = (rawTemp[0] << 8 | rawTemp[1]) & 0x1FFF;

            // Convert into celcius using the formula given in the
            // datasheet. There is a different formula depending on the sign
            // bit, which is the 5th bit of the byte in TEMP_DATA_1.
            if ((rawTemp[0] & (1<<5)) == 0) {
                return rawTempVal / 32.0;
            } else {
                return (rawTempVal - 16384) / 32.0;
            }
        }
    }
    return -1;
}


/**
 * Set the color source. Choices are between internal and external.
 *
 * @param source Internal or External clock
 * @return Success or failure
 */
bool AD5933::setClockSource(byte source) {
    // Determine what source was selected and set it appropriately
    byte newCCL;
    switch (source) {
        case CLOCK_EXTERNAL:
            newCCL = currentControlLow | CTRL_CLOCK_EXTERNAL;
            break;
        case CLOCK_INTERNAL:
            newCCL = currentControlLow & ~CTRL_CLOCK_EXTERNAL;
            break;
        default:
            return false;
    }
    if(newCCL == currentControlLow)
        return true;
    currentControlLow = newCCL;
    return sendByte(CTRL_REG2, newCCL);
}

/**
 * Set the color source to internal or not.
 *
 * @param internal Whether or not to set the clock source as internal.
 * @return Success or failure
 */
bool AD5933::setInternalClock(bool internal) {
    // This function is mainly a wrapper for setClockSource()
    if (internal)
        return setClockSource(CLOCK_INTERNAL);
    else
        return setClockSource(CLOCK_EXTERNAL);
}

/**
 * Set the start frequency for a frequency sweep.
 *
 * @param start The initial frequency.
 * @return Success or failure
 */
bool AD5933::setStartFrequency(unsigned long start) {
    // Page 24 of the Datasheet gives the following formula to represent the
    // start frequency.
    // TODO: Precompute for better performance if we want to keep this constant.
  
    //long freqHex = (increment / (clockSpeed / 4.0))*pow(2, 27);
    long freqHex = (start << 16) / (clockSpeed >> 13);
    if (freqHex > 0xFFFFFF) {
        return false;   // overflow
    }
    currentStartFrequency = start;

    byte bytes[3];
    bytes[0] = freqHex >> 16;
    bytes[1] = freqHex >> 8;
    bytes[2] = freqHex;

    return sendBytes(START_FREQ_1, bytes, 3);
}

/**
 * Set the increment frequency for a frequency sweep.
 *
 * @param start The frequency to increment by. Max of 0xFFFFFF.
 * @return Success or failure
 */
bool AD5933::setIncrementFrequency(unsigned long increment) {
    // Page 25 of the Datasheet gives the following formula to represent the
    // increment frequency.
    // TODO: Precompute for better performance if we want to keep this constant.
    //long freqHex = (increment / (clockSpeed / 4.0))*pow(2, 27);
    long freqHex = (increment << 16) / (clockSpeed >> 13);
    if (freqHex > 0xFFFFFF) {
        return false;   // overflow
    }

    byte bytes[3];
    bytes[0] = freqHex >> 16;
    bytes[1] = freqHex >> 8;
    bytes[2] = freqHex;
    return sendBytes(INC_FREQ_1, bytes, 3);
}

/**
 * Set the number of frequency increments for a frequency sweep.
 *
 * @param start The number of increments to use. Max 511.
 * @return Success or failure
 */
bool AD5933::setNumberIncrements(unsigned int num) {
    // Check that the number sent in is valid.
    if (num > 511) {
        return false;
    }
    numIncrements = num;
    if (num < 1) {
        numIncrements = 0;
        num = 1;
    }
    byte bytes[2];
    bytes[0] = num >> 8;
    bytes[1] = num;
    return sendBytes(NUM_INC_1, bytes, 2);
}

/**
 * Set the PGA gain factor.
 *
 * @param gain The gain factor to select. Use constants or 1/5.
 * @return Success or failure
 */
bool AD5933::setPGAGain(byte gain) {
    // Get the current value of the control register
    byte val = currentControlLow;

    // Clear out the bottom bit, D8, which is the PGA gain set bit
    val &= 0xFE;

    // Determine what gain factor was selected
    if (gain == PGA_GAIN_X1 || gain == 1) {
        // Set PGA gain to x1 in CTRL_REG1
        val |= PGA_GAIN_X1;
    } else if (gain == PGA_GAIN_X5 || gain == 5) {
        // Set PGA gain to x5 in CTRL_REG1
        val |= PGA_GAIN_X5;
    } else {
        return false;
    }
    if(val == currentControlLow)
        return true;
    currentControlLow = val;
    return sendByte(CTRL_REG2, val);
}

/**
 * Set the Voltage excitation range.
 *
 * @param gain The gain factor to select. Use constants or 1/5.
 * @return Success or failure
 */
 bool AD5933::setRange(byte range) {
    // Get the current value of the control register
    byte val = currentControlHigh;

    // Clear out bits, D9 and D10, which is the output voltage range set bits
    val &= 0xF9;
    if (range == VRANGE_1 || range == 0)
    {
        val |= VRANGE_1;
    }
    else if (range == VRANGE_4 || range == 2) {
        val |= VRANGE_4;
    } 
    else if (range == VRANGE_3 || range == 4) {
        val |= VRANGE_3;
    } 
    else if (range == VRANGE_2 || range == 6) {
        val |= VRANGE_2;
    } 
    else {
        return false;
    }
    if(val == currentControlHigh)
        return true;
    currentControlHigh = val;
    return sendByte(CTRL_REG1, val);
}
/**
 * Read the value of a register.
 *
 * @param reg The address of the register to read.
 * @return The value of the register. Returns 0xFF if can't read it.
 */
byte AD5933::readRegister(byte reg) {
    // Read status register and return it's value. If fail, return 0xFF.
    byte val;
    if (getByte(reg, &val)) {
        return val;
    } else {
        return STATUS_ERROR;
    }
}

void AD5933::waitForStatusFlag(byte flag) {
    while(!hasStatusFlagSet(flag)) {}
}

bool AD5933::hasStatusFlagSet(byte flag) {
    return readStatusRegister() & flag;
}

/**
 * Read the value of the status register.
 *
 * @return The value of the status register. Returns 0xFF if can't read it.
 */
byte AD5933::readStatusRegister() {
    lastStatus = readRegister(STATUS_REG);
    return lastStatus;
}

/**
 * Read the value of the control register.
 *
 * @return The value of the control register. Returns 0xFFFF if can't read it.
 */
int AD5933::readControlRegister() {
    return ((readRegister(CTRL_REG1) << 8) | readRegister(CTRL_REG2)) & 0xFFFF;
}

/**
 * Get a raw complex number for a specific frequency measurement.
 *
 * @param real Pointer to an int that will contain the real component.
 * @param imag Pointer to an int that will contain the imaginary component.
 * @param unsafe Whether to enable unsafe speed optimizations
 * @return Success or failure
 */
bool AD5933::getComplexData(int16_t *real, int16_t *imag, bool unsafe) {
    // Wait for a measurement to be available
    uint64_t targetTime = startTime + settlingTime;
    targetTime += unsafe ? 500 : 540;
    sleepMicroseconds(targetTime - esp_timer_get_time());

    if(!unsafe) {
        waitForStatusFlag(STATUS_DATA_VALID);
    }

    byte data[4];
    if(getBytes(REAL_DATA_1, data, 4)) {
        *real = data[1] + (((int16_t)data[0]) << 8);
        *imag = data[3] + (((int16_t)data[2]) << 8);
        return true;
    } else {
        *real = -1;
        *imag = -1;
        return false;
    }
}

bool AD5933::powerOn() {
    return true;
}

bool AD5933::setToStandby() {
    return powerOn() && setPowerMode(POWER_ON);
}

bool AD5933::powerOff() {
    return setPowerMode(POWER_OFF);
}

/**
 * Set the power level of the AD5933.
 *
 * @param level The power level to choose. Can be on, standby, or down.
 * @return Success or failure
 */
bool AD5933::setPowerMode(byte level) {
    // Make the appropriate switch. TODO: Does no operation even do anything?
    currentFrequency = 0;
    switch (level) {
        case POWER_ON:
            return setControlMode(CTRL_STANDBY_MODE);
        case POWER_OFF:
            return setControlMode(CTRL_POWER_DOWN_MODE);
        default:
            return false;
    }
}

void AD5933::updateStartTime() {
    startTime = esp_timer_get_time();
}

bool AD5933::startFrequencySweep() {
    bool success;
    if(currentFrequency == currentStartFrequency) {
        success = setControlMode(CTRL_REPEAT_FREQ);
    } else {
        success = setToStandby() &&
                  setControlMode(CTRL_INIT_START_FREQ) &&
                  setControlMode(CTRL_START_FREQ_SWEEP);
    }
    currentFrequency = currentStartFrequency;
    settlingTime = (uint64_t)currentSettlingCycles * 1000000 / currentStartFrequency;
    sweepIndex = 0;
    updateStartTime();
    return success;
}

void AD5933::setDataStore(int16_t *re, int16_t *im) {
    store_re = re;
    store_im = im;
}

bool AD5933::advanceFrequencySweep(bool unsafe) {
    if(sweepIndex < 0 || sweepIndex > numIncrements) return true;
    
    if (!getComplexData(&store_re[sweepIndex], &store_im[sweepIndex], unsafe)) {
        reset();
        return true;
    }

    // this check may be unnecessary
    if(!unsafe && (lastStatus & STATUS_SWEEP_DONE)) {
        return true;
    }

    // Increment the frequency and our index.
    if(++sweepIndex > numIncrements) {
        return true;
    }

    currentFrequency = 0;
    setControlMode(CTRL_INCREMENT_FREQ);
    updateStartTime();

    return false;
}

bool AD5933::frequencySweep(int16_t real[], int16_t imag[], int n) {
    return frequencySweep(real, imag, n, false);
}

/**
 * Perform a complete frequency sweep.
 *
 * @param real An array of appropriate size to hold the real data.
 * @param imag An array of appropriate size to hold the imaginary data.
 * @param n Length of the array (or the number of discrete measurements)
 * @param unsafe whether to enable unsafe speed optimizations
 * @return Success or failure
 */
bool AD5933::frequencySweep(int16_t real[], int16_t imag[], int n, bool unsafe) {
    // Begin by issuing a sequence of commands
    // If the commands aren't taking hold, add a brief delay
    if(n != numIncrements + 1) return false;

    if(!startFrequencySweep()) {
        reset();
        return false;
    }
    setDataStore(real, imag);

    while (!advanceFrequencySweep(unsafe)) ;
    return sweepIndex >= 0;
}


bool AD5933::setSettlingCycles(int n)
{
  if(n == currentSettlingCycles) return true;
  byte bytes[2];
  bytes[0] = n >> 8;
  bytes[1] = n;
  currentSettlingCycles = n;
  return sendBytes(NUM_SCYCLES_1, bytes, 2);
}

/**
 * Computes the gain factor and phase for each point in a frequency sweep.
 * The gain factor is stored as (1 / GainFactor) for easier later calculation.
 *
 * @param gain An array of appropriate size to hold the gain factors
 * @param phase An array of appropriate size to hold phase data.
 * @param ref The known reference resistance.
 * @param n Length of the array (or the number of discrete measurements)
 * @return Success or failure
 */
bool AD5933::calibrate(long inv_gain[], int phase[], long ref, int n) {
    // We need arrays to hold the real and imaginary values temporarily
    int16_t real[n];
    int16_t imag[n];

    int cycles = currentSettlingCycles;
    setSettlingCycles(60);
    // Perform the frequency sweep
    bool success = frequencySweep(real, imag, n);
    if(success) {
        // For each point in the sweep, calculate the gain factor and phase
        for (int i = 0; i < n; i++) {
            long re = real[i];
            long im = imag[i];
            inv_gain[i] = fast_int_sqrt(re * re + im * im) * ref;
            ESP_LOGI(TAG, "R=%ld/I=%ld, inv_gain=%ld", re, im, inv_gain[i]);
            // TODO: phase
        }
    }

    setSettlingCycles(cycles);
    return success;
}

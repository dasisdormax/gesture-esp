#ifndef AD5933_h
#define AD5933_h

/**
 * All included Files
 */
#include "I2CDevice.h"


// Device address on I2C Bus

//////// USER CONSTANTS ////////
// Temperature measuring
#define TEMP_MEASURE    (CTRL_TEMP_MEASURE)
#define TEMP_NO_MEASURE (CTRL_NO_OPERATION)
// Clock sources
#define CLOCK_INTERNAL  (CTRL_CLOCK_INTERNAL)
#define CLOCK_EXTERNAL  (CTRL_CLOCK_EXTERNAL)
// PGA gain options
#define PGA_GAIN_X1     (CTRL_PGA_GAIN_X1)
#define PGA_GAIN_X5     (CTRL_PGA_GAIN_X5)
// Power modes
#define POWER_OFF       (CTRL_POWER_DOWN_MODE)
#define POWER_ON        (CTRL_STANDBY_MODE)
#define POWER_STANDBY   (CTRL_STANDBY_MODE)

// Control register options
#define CTRL_NO_OPERATION       (0b00000000)
#define CTRL_INIT_START_FREQ    (0b00010000)
#define CTRL_START_FREQ_SWEEP   (0b00100000)
#define CTRL_INCREMENT_FREQ     (0b00110000)
#define CTRL_REPEAT_FREQ        (0b01000000)
#define CTRL_TEMP_MEASURE       (0b10010000)
#define CTRL_POWER_DOWN_MODE    (0b10100000)
#define CTRL_STANDBY_MODE       (0b10110000)
#define CTRL_RESET              (0b00010000)
#define CTRL_CLOCK_EXTERNAL     (0b00001000)
#define CTRL_CLOCK_INTERNAL     (0b00000000)
#define CTRL_PGA_GAIN_X1        (0b00000001)
#define CTRL_PGA_GAIN_X5        (0b00000000)
// Status register options
#define STATUS_TEMP_VALID       (0x01)
#define STATUS_DATA_VALID       (0x02)
#define STATUS_SWEEP_DONE       (0x04)
#define STATUS_ERROR            (0xFF)
// Frequency sweep parameters
#define SWEEP_DELAY             (1)
// Excitation Voltage Range
#define VRANGE_1                 (0b00000000)
#define VRANGE_4                 (0b00000010)
#define VRANGE_3                 (0b00000100)
#define VRANGE_2                 (0b00000110)

//////// CLASS DEFINITION ////////

class AD5933 : protected I2CDevice {
    public:

        // Reset the board
        bool reset(void);

        // Temperature measuring
        bool enableTemperature(byte);
        double getTemperature(void);

        // Clock
        bool setClockSource(byte);
        bool setInternalClock(bool);
        bool setSettlingCycles(int);

        // Frequency sweep configuration
        bool setStartFrequency(unsigned long);
        bool setIncrementFrequency(unsigned long);
        bool setNumberIncrements(unsigned int);

        // Gain configuration
        bool setPGAGain(byte);

        // Excitation range configuration
        bool setRange(byte); // not implemented - not used yet

        // Read registers
	void waitForStatusFlag(byte flag);
        bool hasStatusFlagSet(byte flag);
        byte readRegister(byte address);
        byte readStatusRegister(void);
        int readControlRegister(void);


        // Set control mode register (CTRL_REG1)
        bool setControlMode(byte);

        // Power mode
	bool setToStandby();
	bool powerOn();
	bool powerOff();

        // Perform asynchronous frequency sweeps
        void setDataStore(int16_t *re, int16_t *im);
        bool startFrequencySweep();
        bool advanceFrequencySweep(bool unsafe);

        // Perform frequency sweeps
        bool frequencySweep(int16_t real[], int16_t imag[], int n, bool unsafe);
        bool frequencySweep(int16_t real[], int16_t imag[], int n);
        bool calibrate(long inv_gain[], int phase[], long ref, int n);
        bool calibrate(long inv_gain[], int phase[], int real[],
                              int imag[], int ref, int n);

	AD5933(i2c_port_t port, byte addr);
    private:
        // Private data
        static const unsigned int clockSpeed = 16776000;
	byte currentControlLow;
	byte currentControlHigh;
	unsigned long currentStartFrequency;
        unsigned long currentFrequency;
	int currentSettlingCycles;
        int numIncrements;
	byte lastStatus;
	bool ready;
        int16_t *store_re;
        int16_t *store_im;
        int sweepIndex;

        void updateStartTime();
        uint64_t startTime;
        uint64_t settlingTime;

        // sets the sub-address for the next read or block write
        bool setAddress(byte address);

        // Impedance data
        bool getComplexData(int16_t *, int16_t *, bool unsafe);

	// Power mode
        bool setPowerMode(byte);

        // Sending/Receiving byte method, for easy re-use
        bool getByte(byte address, byte* value);
        bool getBytes(byte address, byte* values, byte numValues);
        bool sendByte(byte address, byte value);
        bool sendBytes(byte address, byte* values, byte numValues);
};

#endif

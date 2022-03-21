# ESP32-based gesture recognition board

This code reads impedance, resistor and IMU values periodically and sends the readings via WiFi to the phone app nearby.

## Hardware

This program was made for an ESP32-based data acquisition system for gesture recognition. The system was designed as part of a Master Thesis and contains an AD5933 impedance analyzer, an IMU using the LIS2MDL and LSM6DSO chips, and a custom resistance measurement circuit using the AD7680 ADC. More details and the thesis are available on request.

The measurements and some processing are done as part of this ESP32 program. The results are sent:
- to the connected PC through UART / USB
- to an Android Smartphone App through WiFi

## Use and Licensing

This code is part of research and can not be considered ready for production. The sensor processing, performance, and power consumption still need to be improved.

Feel free to take inspiration for your own code or use it according to the BSD 3-clause License.

## Instructions for compiling and running

1. Install ESP-IDF, version 4.3 and set up the IDF\_PATH environment variable.
2. Plug in the ESP32 module.
3. `./run.sh`

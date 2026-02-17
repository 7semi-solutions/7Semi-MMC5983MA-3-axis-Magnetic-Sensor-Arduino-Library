# 7Semi-MMC5983MA-3-axis-Magnetic-Sensor-Arduino-Library
Arduino library for the MEMSIC MMC5983MA 3-axis magnetometer.

This library provides easy I2C communication, 18-bit magnetic field readings, temperature measurement, SET/RESET control, and heading helper functions for compass-based applications.

---

## Features

- I2C communication support

- 18-bit X, Y, Z magnetometer readings

- Temperature readout

- Automatic and manual SET/RESET control

- 2D heading calculation

- Tilt-compensated heading helper

- Optional smoothing (EMA filter)

- Example sketches included

---

## Installation

### Method 1 – Install from ZIP

- Download this repository as a ZIP file.

- Open Arduino IDE.

- Go to Sketch → Include Library → Add .ZIP Library

- Select the downloaded ZIP file.

### Method 2 – Manual Install

- Download or clone this repository.

- Copy the folder into:

- Documents/Arduino/libraries/

- Restart Arduino IDE.

---

## Hardware Connections (I2C)

| MMC5983MA | Arduino |
| --------- | ------- |
| VCC       | 3.3V    |
| GND       | GND     |
| SDA       | SDA     |
| SCL       | SCL     |

---

Library Overview
### Initialization
bool begin();

### Read Magnetometer
bool readMagnetometerRaw(int32_t &x, int32_t &y, int32_t &z);

### Read Temperature
bool readTemperature(float &tempC);

### Heading Helpers
float getHeading2D(float declination_deg);
float getHeadingTilt(float roll_deg, float pitch_deg, float declination_deg);


### Notes

Tilt-compensated heading requires external roll and pitch values from an IMU.

For best accuracy, perform magnetic calibration in your application.

Ensure proper sensor alignment with your board axes.



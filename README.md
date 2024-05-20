# KXTJ3-1057 Interface Library

Minimalistic library for motion detection using low cost KXTJ3-1057, 3-axis MEMS accelerometer, low-power, ±2g/±4g/±8g/±16g full scale, high-speed I2C digital output, delivered in a 2x2x0.9mm LGA plastic package operating from a 1.71V–3.6V DC supply.

[![GitHub version](https://img.shields.io/github/release/ldab/KXTJ3-1057.svg)](https://github.com/ldab/KXTJ3-1057/releases/latest)
![Build Status](https://github.com/ldab/KXTJ3-1057/actions/workflows/workflow.yml/badge.svg)
[![License: GPL v3](https://img.shields.io/badge/License-MIT-green.svg)](https://github.com/ldab/KXTJ3-1057/blob/master/LICENSE)

[![GitHub last commit](https://img.shields.io/github/last-commit/ldab/KXTJ3-1057.svg?style=social)](https://github.com/ldab/KXTJ3-1057)

##  Current consumption of operating modes μA

Operating mode (HZ) | Low Power | High Resolution
----------------|-------------------|-----------
0.781|1.543|
1.563|1.635|
3.125|1.922|
6.25|2.488|
12.5|3.431|
25|5.784|
50|9.821|
100|18.15|
200|34.72|
400||156
800||156
1600||156

## Library Features

+ Supports all acceleration ranges and sample rates provided by the IMU
+ Supports low-power 8-bit operation
+ Supports high-resolution 12-bit and 14-bit operation
+ Supports latched, pulsed, and unlatched interrupt modes
+ Supports both "New Data Ready" and "Motion Detection" interrupt modes
+ Support for disabling individual directions in "Motion Detection" mode
+ Support for disabling physical interrupt pin if strict I2C operation is desired
+ Handles IMU power-on sequence and provides correct start-up delays for seamless code integration

## Function Descriptions

### KXTJ3 (addr)

This is the constructor. addr can be either `0x0E` or `0x0F` depending on ADDR pin configuration (please see Table 6 of the datasheet).
Call this with `KXTJ3 myIMU(0x0E)` where myIMU is whatever name you wish to use for that instance of KXTJ3.

### begin(sampleRate, accRange, highResMode, debugMode)

Call this to initialize the IMU. `sampleRate` and `accRange` are required. `highResMode` and `debugMode` default to `false`.
Sample Rates can be found in the power consumption table above. Accelerometer ranges are `2`g/`4`g/`8`g/`16`g.
If `highResMode` is `false` the IMU will use 8-bit low power mode. If `true`, it will use 12-bit high-resolution mode.
If `debugMode` is `true` then debug messages will be printed to the serial monitor for all IMU communication. By default Debug Mode uses `Serial` for output; this can be changed in `kxtj3-1057.cpp` by changing the `#define KXTJ3_DEBUG Serial` line.

### enable14Bit(accRange)

Call this to switch the IMU to 14-bit operation mode. 14-bit operation is only available with `8`g or `16`g ranges.
An example sketch is provided to demonstrate the use of 14-bit mode for the 16g range.

### standby(enabled)

This puts the IMU into 0.9 μA standby mode. Defaults to `true`. Set to `false` to take out of standby.

### axisAccel(axis)

This returns a `float` with the g reading of the specified `axis`. Valid axes are `X`, `Y`, and `Z`.

### readRegister(outputPointer, offset)

Reads the contents of an 8-bit register at address `offset` into a `uint8_t` variable `outputPointer`. All register names are preceded by `KXTJ3_` Please see the datasheet for the names of all available registers that can be passed to `offset`. The basic sketch demonstrates this function by reading the IMU's `KXTJ3_WHO_AM_I` register, as well as by reading acceleration data registers.

### readRegisterInt16(outputPointer, offset)

Reads the contents of two sequential 8-bit registers starting at address `offset` into a single `int16_t` variable `outputPointer`. Very useful for reading the raw contents of axis acceleration data registers in high-resolution mode as this will handle 2's complement correctly. Please see the datasheet for the names of all available registers that can be passed to `offset`. The 14-Bit Mode example sketch also demonstrates proper use of this function to read axis acceleration data.

Please note that if the IMU is in 8-bit low power mode, `readRegister` should be used instead to read only the high byte register for each axis, followed by casting to `int8_t` to get the correct signed value. This is because the lower byte register may contain junk data in 8-bit low power mode.

### writeRegister(offset, dataToWrite)

Writes a single 8-bit value `dataToWrite` to the 8-bit register specified in `offset`. Please see the datasheet for the names of all available registers that can be passed to `offset`. Also note that certain registers can only be changed if the IMU is in standby mode. As changing these special registers is handled by functions exposed by this library, we recommend using this library's functions to change configuration bytes rather than directly writing to configuration registers.

### intConf(moveThreshold, moveDuration, naDuration, polarity, wuRate, latched, pulsed, motion, dataReady, intPin)

This function initializes the IMU's Interrupt Engine. An overview of the system can be found below in the Interrupt Engine section, while a more in-depth description can be found in the datasheet. Example sketches for various configurations are also provided. The parameters for this function are as follows:

+ `moveThreshold`: See Motion Threshold section below. Accepted values are `-2048` through `2047`.
+ `moveDuration`: See Motion Duration section below. Accepted values are `1` through `255`.
+ `naDuration`: See Non-Activity Duration section below. Accepted values are `1` through `255`.
+ `polarity`: Sets whether the INT pin will be active `LOW` or active `HIGH`.
+ `wuRate`: Sets the sample rate for the motion detection function. Valid values are the same as the IMU up to `100` Hz. Defaults to `-1` to lock sample rate to the IMU's sample rate (up to 100 Hz).
+ `latched`: Set to `true` to use latched interrupt mode. Defaults to `false` for unlatched operation. See Table 15 of the datasheet for more details.
+ `pulsed`: Set to `true` to use pulsed interrupt mode. Defaults to `false` for unlatched operation. See Table 15 of the datasheet for more details.
+ `motion`: Set to `false` to disable the Motion Detection function. Defaults to `true`.
+ `dataReady`: Set to `true` to enable using the interrupt engine to signal availability of new acceleration data. Defaults to `false`.
+ `intPin`: Set to `false` to disable triggering the INT pin for interrupts and use only I2C. Defaults to `true`.

### intDisableAxis(interruptDirection)

Allows you to selectively disable individual directions from triggering the Motion Detection interrupt, or reenable all directions. This function must be called **after** `intConf` to have any effect. Valid values are `XNEG`, `XPOS`, `YNEG`, `YPOS`, `ZNEG`, `ZPOS`, and `NONE`. Accepts up to five comma-separated values. Including `NONE` at any point will enable all directions regardless of other parameters. To disable all directions, please set the `motion` parameter in `intConf` to `false` instead.

### motionDetected()

Returns `true` or `false` if the Motion Detection interrupt has been triggered (if enabled).

### motionDirection()

If the Motion Detection interrupt has been triggered, this will return which direction triggered the interrupt. Return values are `XNEG`, `XPOS`, `YNEG`, `YPOS`, `ZNEG`, `ZPOS`, or `NONE` if the interrupt has not been triggered or has been reset.

### dataReady()

Returns `true` or `false` if the New Data Ready interrupt has been triggered (if enabled).

### resetInterrupt()

Resets the interrupt latch if the Interrupt Engine is in latched operation mode.

## Error Handler

This library includes an I2C error handler on all functions except `axisAccel`, `dataReady`, `motionDetected`, and `motionDirection`. Return values of all other functions are either `IMU_SUCCESS` or `IMU_HW_ERROR`. All example sketches include the use of error handling for read/write functions.

## Interrupt Engine

### Motion Threshold

Interrupt threshold sensitivity is compared to the top 12bits of the accelerometer 8g output value regardless of the resolution chosen. This value can be anything from `-2048` to `2047`.

* i.e -8g (-2048/256) to 0.0039g (1/256) to 8g (2047/256)

### Motion Duration

This is the amount of time that the change in acceleration must be above the Motion Threshold before the Motion Detection interrupt will be triggered, and is a function of Duration and Sample Rate.

Please note that in the context of Interrupt Duration, only Sample Rates up to 100 Hz are supported. If the IMU is set to a Sample Rate greater than 100 Hz and the Motion Detection Sample Rate is locked to IMU Sample Rate, Interrupt Duration will use 100 Hz for its calculation.

> This value can be anything from 1 to 255

* i.e. 5 event_counts / 6.25 Hz = 0.8 seconds

### Non-Activity Duration

This is the amount of time that the change in acceleration must be below the Motion Threshold before another Motion Detection interrupt can be triggered, and is a function of Duration and Sample Rate.

As with Interrupt Duration, please note that only Sample Rates up to 100 Hz are supported. If the IMU is set to a Sample Rate greater than 100 Hz and the Motion Detection Sample Rate is locked to IMU Sample Rate, Non-Activity Duration will use 100 Hz for its calculation.

> This value can be anything from 1 to 255

* i.e. 5 event_counts / 6.25 Hz = 0.8 seconds

## Credits

Github Shields and Badges created with [Shields.io](https://github.com/badges/shields/)
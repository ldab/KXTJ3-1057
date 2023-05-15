# KXTJ3-1057 Motion detection

Minimalistic library for motion detection using low cost KXTJ3-1057, 3-axis MEMS accelerometer, low-power, ±2g/±4g/±8g/±16g full scale, high-speed I2C digital output, delivered in a 2x2x0.9mm LGAplastic package operating from a 1.71V–3.6V DC supply.

[![GitHub version](https://img.shields.io/github/release/ldab/KXTJ3-1057.svg)](https://github.com/ldab/KXTJ3-1057/releases/latest)
[![Build Status](https://travis-ci.org/ldab/KXTJ3-1057.svg?branch=master)](https://travis-ci.org/ldab/KXTJ3-1057)
[![License: GPL v3](https://img.shields.io/badge/License-MIT-green.svg)](https://github.com/ldab/KXTJ3-1057/blob/master/LICENSE)

[![GitHub last commit](https://img.shields.io/github/last-commit/ldab/KXTJ3-1057.svg?style=social)](https://github.com/ldab/KXTJ3-1057)

## TODO

- [ ] `returnError` Handler
- [ ] Update Keywords

###  Current consumption of operating modes μA

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

## Interrupt Threshold

This is the sensitivity required to trigger the interrupt (and it must exceed this for duration, see below)

Range from 1 to 4095

Take desried sensitivity in g and multiply by 256

* ex. for 4g, the value should be set to 4 * 256 = 1024

## Interrupt Duration

Interrupt event duration to trigger the interrupt pin is a function of events and Sample Rate:

> This value can be anything from 1 to 255

* i.e 5 event_counts / 6.25 Hz = 0.8 seconds

## Non-Activity Duration

Interrupt non-activity duration to *reset* the interrupt pin is a function of events and Sample Rate:

> This value can be anything from 1 to 255

* i.e 5 event_counts / 6.25 Hz = 0.8 seconds

## Known Limitations

* Only 8 and 12 bits modes are implemented (14-bit is not);
* Only non-latched interrupt is implemented;

## Credits

Github Shields and Badges created with [Shields.io](https://github.com/badges/shields/)
# KXTJ3-1057 Motion detection

Minimalistic library for motion detection using low cost KXTJ3-1057, 3-axis MEMS accelerometer, low-power, ±2g/±4g/±8g/±16g full scale, high-speed I2C digital output, delivered in a 2x2x0.9mm LGAplastic package operating from a 1.71V–3.6V DC supply.

[![GitHub version](https://img.shields.io/github/release/ldab/KXTJ3-1057.svg)](https://github.com/ldab/KXTJ3-1057/releases/latest)
[![Build Status](https://travis-ci.org/ldab/KXTJ3-1057.svg?branch=master)](https://travis-ci.org/ldab/KXTJ3-1057)
[![License: GPL v3](https://img.shields.io/badge/License-MIT-green.svg)](https://github.com/ldab/KXTJ3-1057/blob/master/LICENSE)

[![GitHub last commit](https://img.shields.io/github/last-commit/ldab/KXTJ3-1057.svg?style=social)](https://github.com/ldab/KXTJ3-1057)

## TODO

- [ ] Add Filter options
- [ ] Update Keywords

## Low Power configuration

Operating mode  | Turn-on time (ms) | @2g (mg/digit)  
----------------|-------------------|----------------
Low Power 8bits |1|16
Normal Mode 10bits|1.6|4
High Res 12bits|7/ODD|1

###  Current consumption of operating modes μA

Operating mode (HZ) | Low Power | Normal Mode | High Resolution
----------------|-------------------|----------------|-----------
1|2|2|2
10|3|4|4
25|4|6|6
50|6|11|11
100|10|20|20
200|18|38|38
400|36|73|73
1344||185|185
1620|100||

## Interrupt Threshold

Interrupt threshold sensitivity is directly proportional to the Full Scale 2, 4, 8 or 16g chosen:

> This value can be anything from 1 to 127

* 16 mg @ FS = ±2 g
* 32 mg @ FS = ±4 g
* 62 mg @ FS = ±8 g
* 186 mg @ FS = ±16 g

## Known Limitations

* Only 8 and 12 bits modes are implemented (14-bit is not);

## Credits

Github Shields and Badges created with [Shields.io](https://github.com/badges/shields/)
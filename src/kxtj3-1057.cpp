/******************************************************************************
KXTJ3-1057.cpp
KXTJ3-1057 for Arduino
Leonardo Bispo & Nomake Wan
June, 2023
https://github.com/ldab/KXTJ3-1057
Resources:
Uses Wire.h for I2C operation

Distributed as-is; no warranty is given.
******************************************************************************/

// Changes which Serial port to use for debug messages (Serial0, Serial1, etc)
#define KXTJ3_DEBUG Serial

#include "kxtj3-1057.h"
#include "stdint.h"

#include "Wire.h"

//****************************************************************************//
//
//  Default construction is I2C mode, address 0x0E.
//
//****************************************************************************//
KXTJ3::KXTJ3(uint8_t inputArg = 0x0E) { I2CAddress = inputArg; }

kxtj3_status_t KXTJ3::begin(float sampleRate, uint8_t accRange, bool highResSet,
                            bool debugSet)
{
  kxtj3_status_t returnError = IMU_SUCCESS;
  accelSampleRate            = sampleRate;
  accelRange                 = accRange;
  highRes                    = highResSet;
  en14Bit                    = false;
  debugMode                  = debugSet;

  if (debugMode) {
    KXTJ3_DEBUG.println(F("Configuring IMU"));
  }

  // Sample rates ≥ 400Hz force High Resolution mode on
  if (accelSampleRate > 200 && !highRes) {
    highRes = true;
  }

  Wire.begin();

  // Power-up time is up to 30ms according to DataSheet, so delay 50ms just to
  // play it safe
  delay(50);

  // Perform software reset to make sure IMU is in good state
  returnError = softwareReset();

  // Check previous returnError to see if we should stop
  if (returnError != IMU_SUCCESS) {
    return returnError;
  }

  // Check the ID register to determine if the operation was a success.
  uint8_t _whoAmI;

  readRegister(&_whoAmI, KXTJ3_WHO_AM_I);

  if (_whoAmI != 0x35) {
    return IMU_HW_ERROR;
  }

  // Check the self-test register to determine if the IMU is up.
  uint8_t _selfTest;

  readRegister(&_selfTest, KXTJ3_DCST_RESP);

  if (_selfTest != 0x55) {
    return IMU_HW_ERROR;
  }

  if (debugMode) {
    KXTJ3_DEBUG.println(F("Apply settings"));
  }

  returnError = applySettings();

  return returnError;
}

//****************************************************************************//
//  ReadRegisterRegion
//
//  Parameters:
//    *outputPointer -- Pass &variable (base address of) to save read data to
//    offset -- register to read
//    length -- number of bytes to read
//****************************************************************************//
kxtj3_status_t KXTJ3::readRegisterRegion(uint8_t *outputPointer, uint8_t offset,
                                         uint8_t length)
{
  kxtj3_status_t returnError = IMU_SUCCESS;

  // define pointer that will point to the external space
  uint8_t i                  = 0;
  uint8_t c                  = 0;

  Wire.beginTransmission(I2CAddress);
  offset |= 0x80; // turn auto-increment bit on, bit 7 for I2C
  Wire.write(offset);
  if (Wire.endTransmission() != 0) {
    return IMU_HW_ERROR;
  } else // OK, all worked, keep going
  {
    // request 6 bytes from slave device
    Wire.requestFrom(I2CAddress, length);
    while ((Wire.available()) &&
           (i < length)) // slave may send less than requested
    {
      c              = Wire.read(); // receive a byte as character
      *outputPointer = c;
      outputPointer++;
      i++;
    }
  }

  return returnError;
}

//****************************************************************************//
//  ReadRegister
//
//  Parameters:
//    *outputPointer -- Pass &variable (address of) to save read data to
//    offset -- register to read
//****************************************************************************//
kxtj3_status_t KXTJ3::readRegister(uint8_t *outputPointer, uint8_t offset)
{
  // Return value
  uint8_t result             = 0;
  uint8_t numBytes           = 1;
  kxtj3_status_t returnError = IMU_SUCCESS;

  Wire.beginTransmission(I2CAddress);
  Wire.write(offset);

  if (Wire.endTransmission() != 0) {
    return IMU_HW_ERROR;
  }

  Wire.requestFrom(I2CAddress, numBytes);

  while (Wire.available()) // slave may send less than requested
  {
    result = Wire.read(); // receive a byte as a proper uint8_t
  }

  if (debugMode) {
    KXTJ3_DEBUG.print(F("Read register 0x"));
    KXTJ3_DEBUG.print(offset, HEX);
    KXTJ3_DEBUG.print(F(" = 0x"));
    KXTJ3_DEBUG.println(result, HEX);
  }

  *outputPointer = result;
  return returnError;
}

//****************************************************************************//
//  readRegisterInt16
//
//  Parameters:
//    *outputPointer -- Pass &variable (base address of) to save read data to
//    offset -- register to read
//****************************************************************************//
kxtj3_status_t KXTJ3::readRegisterInt16(int16_t *outputPointer, uint8_t offset)
{
  // offset |= 0x80; //turn auto-increment bit on
  uint8_t myBuffer[2];
  kxtj3_status_t returnError =
      readRegisterRegion(myBuffer, offset, 2); // Does memory transfer
  int16_t output = (int16_t)myBuffer[0] | int16_t(myBuffer[1] << 8);

  if (debugMode && returnError == IMU_SUCCESS) {
    KXTJ3_DEBUG.print(F("16 bits from 0x"));
    KXTJ3_DEBUG.print(offset, HEX);
    KXTJ3_DEBUG.print(F(" = "));
    KXTJ3_DEBUG.println(output);
  } else if (returnError != IMU_SUCCESS) {
    return returnError;
  }

  *outputPointer = output;
  return returnError;
}

//****************************************************************************//
//  writeRegister
//
//  Parameters:
//    offset -- register to write
//    dataToWrite -- 8 bit data to write to register
//****************************************************************************//
kxtj3_status_t KXTJ3::writeRegister(uint8_t offset, uint8_t dataToWrite)
{
  kxtj3_status_t returnError = IMU_SUCCESS;

  // Write the byte
  Wire.beginTransmission(I2CAddress);
  Wire.write(offset);
  Wire.write(dataToWrite);
  if (Wire.endTransmission() != 0) {
    returnError = IMU_HW_ERROR;
  }

  return returnError;
}

//****************************************************************************//
//
//  softwareReset
//  Resets the device; recommended by Kionix on initial power-up (TN017)
//
//****************************************************************************//
kxtj3_status_t KXTJ3::softwareReset(void)
{
  kxtj3_status_t returnError = IMU_SUCCESS;

  // Start by copying the current I2C address to a temp variable
  // We must do this because the IMU could boot with a bit-flipped address
  uint8_t tempAddress        = I2CAddress;

  // Write 0x00 to KXTJ3_SOFT_REST to confirm IMU is on the bus at address
  Wire.beginTransmission(I2CAddress);
  Wire.write(KXTJ3_SOFT_REST);
  Wire.write(0x00);

  // If NACK returned, switch I2CAddress to flipped version and try again
  if (Wire.endTransmission() != 0) {
    if (I2CAddress == 0x0F) {
      I2CAddress = 0x0D;
    } else if (I2CAddress == 0x0E) {
      I2CAddress = 0x0C;
    }

    Wire.beginTransmission(I2CAddress);
    Wire.write(KXTJ3_SOFT_REST);
    Wire.write(0x00);

    // If still NACK, give up, need to power cycle IMU to recover
    if (Wire.endTransmission() != 0) {
      // Return I2CAddress to normal before returning
      if (I2CAddress != tempAddress) {
        I2CAddress = tempAddress;
      }
      return IMU_HW_ERROR;
    }
  }

  // Attempt to address CTRL_REG2 and end if NACK returned
  Wire.beginTransmission(I2CAddress);
  Wire.write(KXTJ3_CTRL_REG2);
  Wire.write(0x00);

  if (Wire.endTransmission() != 0) {
    // Return I2CAddress to normal before returning
    if (I2CAddress != tempAddress) {
      I2CAddress = tempAddress;
    }
    return IMU_HW_ERROR;
  }

  // Send software reset command to CTRL_REG2 and end if NACK returned
  Wire.beginTransmission(I2CAddress);
  Wire.write(KXTJ3_CTRL_REG2);
  Wire.write(0x80);

  if (Wire.endTransmission() != 0) {
    // Return I2CAddress to normal before returning
    if (I2CAddress != tempAddress) {
      I2CAddress = tempAddress;
    }
    return IMU_HW_ERROR;
  }

  // Set I2CAddress back to normal since we've successfully reset the IMU
  if (I2CAddress != tempAddress) {
    I2CAddress = tempAddress;
  }

  // Delay for software start-up before returning (TN017 Table 1)
  delay(2);

  return returnError;
}

//****************************************************************************//
//
//  Read axis acceleration as Float
//
//****************************************************************************//
float KXTJ3::axisAccel(axis_t _axis)
{
  int16_t outRAW;
  uint8_t regToRead = 0;
  switch (_axis) {
  case 0:
    // X axis
    regToRead = KXTJ3_XOUT_L;
    break;
  case 1:
    // Y axis
    regToRead = KXTJ3_YOUT_L;
    break;
  case 2:
    // Z axis
    regToRead = KXTJ3_ZOUT_L;
    break;

  default:
    // Not valid axis return NAN
    return NAN;
    break;
  }

  // Don't proceed if the read failed
  if (readRegisterInt16(&outRAW, regToRead) != IMU_SUCCESS) {
    return NAN;
  }

  // The LSB may contain garbage, so 0 any unused bits
  if (!highRes) {
    outRAW &= 0b1111111100000000; // 8-bit mode
  } else if (en14Bit) {
    outRAW &= 0b1111111111111100; // 14-bit mode
  } else {
    outRAW &= 0b1111111111110000; // 12-bit mode
  }

  float outFloat;

  switch (accelRange) {
  case 2:
    outFloat = (float)outRAW / 16384;
    break;
  case 4:
    outFloat = (float)outRAW / 8192;
    break;
  case 8:
    outFloat = (float)outRAW / 4096;
    break;
  case 16:
    outFloat = (float)outRAW / 2048;
    break;
  default:
    outFloat = 0;
    break;
  }

  return outFloat;
}

//****************************************************************************//
//
//  Place the accelerometer into/out of standby
//
//****************************************************************************//
kxtj3_status_t KXTJ3::standby(bool _en)
{
  kxtj3_status_t returnError = IMU_SUCCESS;
  uint8_t _ctrl;

  // "Backup" KXTJ3_CTRL_REG1
  returnError = readRegister(&_ctrl, KXTJ3_CTRL_REG1);

  if (returnError != IMU_SUCCESS) {
    return returnError;
  }

  if (_en)
    _ctrl &= 0x7E;
  else
    _ctrl |= (0x01 << 7); // disable standby-mode -> Bit7 = 1 = operating mode

  returnError = writeRegister(KXTJ3_CTRL_REG1, _ctrl);

  // If taking out of standby, follow start-up delay
  if (!_en && returnError == IMU_SUCCESS) {
    startupDelay();
  }

  return returnError;
}

//****************************************************************************//
//
//  Applies the start-up delay specified in Table 1 of the DataSheet
//  Used when coming out of standby or softwareReset
//
//****************************************************************************//
void KXTJ3::startupDelay(void)
{
  if (highRes) {
    if (accelSampleRate < 1)
      delay(1300);
    else if (accelSampleRate < 3)
      delay(650);
    else if (accelSampleRate < 6)
      delay(330);
    else if (accelSampleRate < 12)
      delay(170);
    else if (accelSampleRate < 25)
      delay(90);
    else if (accelSampleRate < 50)
      delay(45);
    else if (accelSampleRate < 100)
      delay(25);
    else if (accelSampleRate < 200)
      delay(11);
    else if (accelSampleRate < 400)
      delay(6);
    else if (accelSampleRate < 800)
      delay(4);
    else if (accelSampleRate < 1600)
      delay(3);
    else
      delay(2);
  } else {
    if (accelSampleRate < 800 && accelSampleRate > 200)
      delay(4);
    else if (accelSampleRate < 1600 && accelSampleRate > 400)
      delay(3);
    else
      delay(2);
  }
}

//****************************************************************************//
//
//  Apply settings passed to .begin();
//
//****************************************************************************//
kxtj3_status_t KXTJ3::applySettings(void)
{
  kxtj3_status_t returnError = IMU_SUCCESS;
  uint8_t dataToWrite        = 0; // Temporary variable

  // Note that to properly change the value of this register, the PC1 bit in
  // CTRL_REG1 must first be set to “0”.
  returnError = standby(true);

  if (returnError != IMU_SUCCESS) {
    return returnError;
  }

  // Build DATA_CTRL_REG

  //  Convert ODR
  if (accelSampleRate < 1)
    dataToWrite |= 0x08; // 0.781Hz
  else if (accelSampleRate < 2)
    dataToWrite |= 0x09; // 1.563Hz
  else if (accelSampleRate < 4)
    dataToWrite |= 0x0A; // 3.125Hz
  else if (accelSampleRate < 8)
    dataToWrite |= 0x0B; // 6.25Hz
  else if (accelSampleRate < 16)
    dataToWrite |= 0x00; // 12.5Hz
  else if (accelSampleRate < 30)
    dataToWrite |= 0x01; // 25Hz
  else if (accelSampleRate < 60)
    dataToWrite |= 0x02; // 50Hz
  else if (accelSampleRate < 150)
    dataToWrite |= 0x03; // 100Hz
  else if (accelSampleRate < 250)
    dataToWrite |= 0x04; // 200Hz
  else if (accelSampleRate < 450)
    dataToWrite |= 0x05; // 400Hz
  else if (accelSampleRate < 850)
    dataToWrite |= 0x06; // 800Hz
  else
    dataToWrite |= 0x07; // 1600Hz

  // Now, write the patched together data
  if (debugMode) {
    KXTJ3_DEBUG.print(F("KXTJ3_DATA_CTRL_REG: 0x"));
    KXTJ3_DEBUG.println(dataToWrite, HEX);
  }
  returnError = writeRegister(KXTJ3_DATA_CTRL_REG, dataToWrite);

  if (returnError != IMU_SUCCESS) {
    return returnError;
  }

  // Build CTRL_REG1

  // LOW power, 8-bit mode
  dataToWrite = 0x80;

  if (highRes) {
    if (debugMode) {
      KXTJ3_DEBUG.println(F("High Resolution set"));
    }
    dataToWrite = 0xC0;
  }

  //  Convert scaling
  switch (accelRange) {
  default:
  case 2:
    dataToWrite |= (0x00 << 2);
    break;
  case 4:
    dataToWrite |= (0x02 << 2);
    break;
  case 8:
    dataToWrite |= (0x04 << 2);
    break;
  case 16:
    dataToWrite |= (0x01 << 2);
    break;
  }

  // Now, write the patched together data
  if (debugMode) {
    KXTJ3_DEBUG.print(F("KXTJ3_CTRL_REG1: 0x"));
    KXTJ3_DEBUG.println(dataToWrite, HEX);
  }
  returnError = writeRegister(KXTJ3_CTRL_REG1, dataToWrite);
  startupDelay();
  return returnError;
}

//****************************************************************************//
//
//  Enables 14-bit operation mode for 8g/16g acceleration ranges
//
//****************************************************************************//
kxtj3_status_t KXTJ3::enable14Bit(uint8_t accRange)
{
  kxtj3_status_t returnError = IMU_SUCCESS;
  uint8_t dataToWrite        = 0;        // Temporary variable
  accelRange                 = accRange; // Set accelRange to new value
  highRes                    = true;     // Make sure highRes is set to true
  en14Bit                    = true;     // Set 14-bit check to true as well

  // Note that to properly change the value of this register, the PC1 bit in
  // CTRL_REG1 must first be set to “0”.
  returnError = standby(true);

  if (returnError != IMU_SUCCESS) {
    return returnError;
  }

  if (debugMode) {
    KXTJ3_DEBUG.println(F("Switching to 14-bit mode"));
  }

  // Build CTRL_REG1

  switch (accelRange) {
  default:
  case 16:
    dataToWrite = 0xDC;
    break;
  case 8:
    dataToWrite = 0xD8;
    break;
  }

  // Write the new data to CTRL_REG1
  if (debugMode) {
    KXTJ3_DEBUG.print(F("KXTJ3_CTRL_REG1: 0x"));
    KXTJ3_DEBUG.println(dataToWrite, HEX);
  }
  returnError = writeRegister(KXTJ3_CTRL_REG1, dataToWrite);

  if (returnError == IMU_SUCCESS) {
    startupDelay();
  }

  return returnError;
}

//****************************************************************************//
//  Configure interrupt, stop or move, threshold and duration
//	Duration, steps and maximum values depend on the ODR chosen.
//****************************************************************************//
kxtj3_status_t KXTJ3::intConf(int16_t threshold, uint8_t moveDur, uint8_t naDur,
                              bool polarity, float wuRate, bool latched,
                              bool pulsed, bool motion, bool dataReady,
                              bool intPin)
{
  kxtj3_status_t returnError = IMU_SUCCESS;

  // Note that to properly change the value of this register, the PC1 bit in
  // CTRL_REG1 must first be set to “0”.
  returnError = standby(true);

  if (returnError != IMU_SUCCESS) {
    return returnError;
  }

  // Build INT_CTRL_REG1

  uint8_t dataToWrite = 0x00; // Interrupt pin disabled, active LOW, latched

  // uint8_t dataToWrite = 0x20; // Interrupt enabled, active LOW, latched

  if (pulsed)
    dataToWrite |= (0x01 << 3); // Interrupt pin pulsed

  if (polarity == HIGH)
    dataToWrite |= (0x01 << 4); // Active HIGH

  if (intPin)
    dataToWrite |= (0x01 << 5); // Interrupt pin enabled

  if (debugMode) {
    KXTJ3_DEBUG.print(F("KXTJ3_INT_CTRL_REG1: 0x"));
    KXTJ3_DEBUG.println(dataToWrite, HEX);
  }

  returnError = writeRegister(KXTJ3_INT_CTRL_REG1, dataToWrite);

  if (returnError != IMU_SUCCESS) {
    return returnError;
  }

  uint8_t _reg1;

  // First "back up" current settings to a temporary variable
  returnError = readRegister(&_reg1, KXTJ3_CTRL_REG1);

  if (returnError != IMU_SUCCESS) {
    return returnError;
  }

  if (motion)
    _reg1 |= (0x01 << 1); // Sets WUFE to enabled

  if (dataReady)
    _reg1 |= (0x01 << 5); // Sets DRDY to enabled

  if (debugMode) {
    KXTJ3_DEBUG.print(F("KXTJ3_CTRL_REG1: 0x"));
    KXTJ3_DEBUG.println(dataToWrite, HEX);
  }

  returnError = writeRegister(KXTJ3_CTRL_REG1, _reg1);

  if (returnError != IMU_SUCCESS) {
    return returnError;
  }

  // Set data rate for Wake-Up Function

  if (wuRate < 0) {
    // Sets the Data Rate for the Wake-Up (motion detect) function to match ODR
    // Start by checking DATA_CTRL_REG for the current ODR

    returnError = readRegister(&_reg1, KXTJ3_DATA_CTRL_REG);

    if (returnError != IMU_SUCCESS) {
      return returnError;
    }

    // Set ODRWU based on ODR
    // Maximum ODRWU is 100 Hz

    switch (_reg1) {
    case 0x09:
      dataToWrite = 0x01; // 1.563 Hz
      break;
    case 0x0A:
      dataToWrite = 0x02; // 3.125 Hz
      break;
    case 0x0B:
      dataToWrite = 0x03; // 6.25 Hz
      break;
    case 0x00:
      dataToWrite = 0x04; // 12.5 Hz
      break;
    case 0x01:
      dataToWrite = 0x05; // 25 Hz
      break;
    case 0x02:
      dataToWrite = 0x06; // 50 Hz
      break;
    case 0x03:
    case 0x04:
    case 0x05:
    case 0x06:
    case 0x07:
      dataToWrite = 0x07; // 100 Hz
      break;
    default:
      dataToWrite = 0x00; // 0.781 Hz
      break;
    }
  } else if (wuRate < 1)
    dataToWrite = 0x00; // 0x781 Hz
  else if (wuRate < 2)
    dataToWrite = 0x01; // 1.563 Hz
  else if (wuRate < 4)
    dataToWrite = 0x02; // 3.125 Hz
  else if (wuRate < 7)
    dataToWrite = 0x03; // 6.25 Hz
  else if (wuRate < 13)
    dataToWrite = 0x04; // 12.5 Hz
  else if (wuRate < 26)
    dataToWrite = 0x05; // 25 Hz
  else if (wuRate < 51)
    dataToWrite = 0x06; // 50 Hz
  else
    dataToWrite = 0x07; // 100 Hz

  if (debugMode) {
    KXTJ3_DEBUG.print(F("KXTJ3_CTRL_REG2: 0x"));
    KXTJ3_DEBUG.println(dataToWrite, HEX);
  }

  returnError = writeRegister(KXTJ3_CTRL_REG2, dataToWrite);

  if (returnError != IMU_SUCCESS) {
    return returnError;
  }

  // Build INT_CTRL_REG2

  dataToWrite = 0x3F; // enable interrupt on all axes any direction, latched

  if (!latched)
    dataToWrite |= (0x01 << 7); // enable unlatched mode

  if (debugMode) {
    KXTJ3_DEBUG.print(F("KXTJ3_INT_CTRL_REG2: 0x"));
    KXTJ3_DEBUG.println(dataToWrite, HEX);
  }

  returnError = writeRegister(KXTJ3_INT_CTRL_REG2, dataToWrite);

  if (returnError != IMU_SUCCESS) {
    return returnError;
  }

  // Set WAKE-UP (motion detect) Threshold

  dataToWrite = (uint8_t)(threshold >> 4);

  if (debugMode) {
    KXTJ3_DEBUG.print(F("KXTJ3_WAKEUP_THRESHOLD_H: 0x"));
    KXTJ3_DEBUG.println(dataToWrite, HEX);
  }

  returnError = writeRegister(KXTJ3_WAKEUP_THRESHOLD_H, dataToWrite);

  if (returnError != IMU_SUCCESS) {
    return returnError;
  }

  dataToWrite = (uint8_t)(threshold << 4);

  if (debugMode) {
    KXTJ3_DEBUG.print(F("KXTJ3_WAKEUP_THRESHOLD_L: 0x"));
    KXTJ3_DEBUG.println(dataToWrite, HEX);
  }

  returnError = writeRegister(KXTJ3_WAKEUP_THRESHOLD_L, dataToWrite);

  if (returnError != IMU_SUCCESS) {
    return returnError;
  }

  // WAKEUP_COUNTER -> Sets the time motion must be present before a wake-up
  // interrupt is set WAKEUP_COUNTER (counts) = Wake-Up Delay Time (sec) x
  // Wake-Up Function ODR(Hz)

  dataToWrite = moveDur;

  if (debugMode) {
    KXTJ3_DEBUG.print(F("KXTJ3_WAKEUP_COUNTER: 0x"));
    KXTJ3_DEBUG.println(dataToWrite, HEX);
  }

  returnError = writeRegister(KXTJ3_WAKEUP_COUNTER, dataToWrite);

  if (returnError != IMU_SUCCESS) {
    return returnError;
  }

  // Non-Activity register sets the non-activity time required before another
  // wake-up interrupt will be reported. NA_COUNTER (counts) = Non-ActivityTime
  // (sec) x Wake-Up Function ODR(Hz)

  dataToWrite = naDur;

  if (debugMode) {
    KXTJ3_DEBUG.print(F("KXTJ3_NA_COUNTER: 0x"));
    KXTJ3_DEBUG.println(dataToWrite, HEX);
  }

  returnError = writeRegister(KXTJ3_NA_COUNTER, dataToWrite);

  if (returnError != IMU_SUCCESS) {
    return returnError;
  }

  // Set IMU to Operational mode
  returnError = standby(false);

  return returnError;
}

kxtj3_status_t KXTJ3::intDisableAxis(wu_axis_t first, wu_axis_t second,
                                     wu_axis_t third, wu_axis_t fourth,
                                     wu_axis_t fifth)
{
  // Create temporary variables
  kxtj3_status_t returnError = IMU_SUCCESS;
  uint8_t temp               = 0x00;
  uint8_t dataToWrite        = 0b00111111;
  uint8_t bitCheck;

  // Note that to properly change the value of this register, the PC1 bit in
  // CTRL_REG1 must first be set to “0”.
  returnError = standby(true);

  if (returnError != IMU_SUCCESS) {
    return returnError;
  }

  // Check to see if ULMODE bit is set and set if so
  returnError = readRegister(&bitCheck, KXTJ3_INT_CTRL_REG2);
  if (returnError != IMU_SUCCESS)
    return returnError;
  if (bitCheck & (0x01 << 7))
    dataToWrite |= (0x01 << 7);

  if (first & NONE || second & NONE || third & NONE || fourth & NONE ||
      fifth & NONE) {
    // Rebuild INT_CTRL_REG2 with 0x00 using XOR to enable all axes
    dataToWrite ^= temp;
  } else {
    // combine the requested axes
    temp |= first;
    temp |= second;
    temp |= third;
    temp |= fourth;
    temp |= fifth;

    // Rebuild INT_CTRL_REG2 with new axis data using XOR
    dataToWrite ^= temp;
  }

  // Write the new values to INT_CTRL_REG2
  if (debugMode) {
    KXTJ3_DEBUG.print(F("KXTJ3_INT_CTRL_REG2: 0x"));
    KXTJ3_DEBUG.println(dataToWrite, HEX);
  }

  returnError = writeRegister(KXTJ3_INT_CTRL_REG2, dataToWrite);

  // Set IMU to Operational mode
  returnError = standby(false);

  return returnError;
}

kxtj3_status_t KXTJ3::intDisableAxis(wu_axis_t first)
{
  return intDisableAxis(first, BLANK, BLANK, BLANK, BLANK);
}

kxtj3_status_t KXTJ3::intDisableAxis(wu_axis_t first, wu_axis_t second)
{
  return intDisableAxis(first, second, BLANK, BLANK, BLANK);
}

kxtj3_status_t KXTJ3::intDisableAxis(wu_axis_t first, wu_axis_t second,
                                     wu_axis_t third)
{
  return intDisableAxis(first, second, third, BLANK, BLANK);
}

kxtj3_status_t KXTJ3::intDisableAxis(wu_axis_t first, wu_axis_t second,
                                     wu_axis_t third, wu_axis_t fourth)
{
  return intDisableAxis(first, second, third, fourth, BLANK);
}

bool KXTJ3::dataReady(void)
{
  uint8_t _reg1;

  readRegister(&_reg1, KXTJ3_INT_SOURCE1);

  // Bit 4 is Data Ready Interrupt Bit
  if (_reg1 & (0x01 << 4)) {
    return true;
  } else {
    return false;
  }
}

bool KXTJ3::motionDetected(void)
{
  uint8_t _reg1;

  readRegister(&_reg1, KXTJ3_INT_SOURCE1);

  // Bit 1 is Wake-Up Function Sense Bit
  if (_reg1 & (0x01 << 1)) {
    return true;
  } else {
    return false;
  }
}

wu_axis_t KXTJ3::motionDirection(void)
{
  uint8_t _reg1;

  readRegister(&_reg1, KXTJ3_INT_SOURCE2);

  if (_reg1 & (0x01 << 0))
    return ZPOS;
  else if (_reg1 & (0x01 << 1))
    return ZNEG;
  else if (_reg1 & (0x01 << 2))
    return YPOS;
  else if (_reg1 & (0x01 << 3))
    return YNEG;
  else if (_reg1 & (0x01 << 4))
    return XPOS;
  else if (_reg1 & (0x01 << 5))
    return XNEG;
  else
    return NONE;
}

kxtj3_status_t KXTJ3::resetInterrupt(void)
{
  kxtj3_status_t returnError = IMU_SUCCESS;

  uint8_t _reg1;

  // Reading the INT_REL register releases the latch
  returnError = readRegister(&_reg1, KXTJ3_INT_REL);

  return returnError;
}

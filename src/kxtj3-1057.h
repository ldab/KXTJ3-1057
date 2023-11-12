/******************************************************************************
KXTJ3-1057.h
KXTJ3-1057 for Arduino
Leonardo Bispo & Nomake Wan
June, 2023
https://github.com/ldab/KXTJ3-1057
Resources:
Uses Wire.h for I2C operation

Distributed as-is; no warranty is given.
******************************************************************************/

#ifndef __KXTJ3_IMU_H__
#define __KXTJ3_IMU_H__

#include "stdint.h"

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

// Print variable name
#define getName(var) #var

// Return values
typedef enum {
  IMU_SUCCESS,
  IMU_HW_ERROR,
  IMU_NOT_SUPPORTED,
  IMU_GENERIC_ERROR,
  IMU_OUT_OF_BOUNDS,
  IMU_ALL_ONES_WARNING,
  //...
} kxtj3_status_t;

typedef enum {
  X = 0,
  Y,
  Z,
} axis_t;

typedef enum {
  BLANK = 0,
  ZPOS  = 1,
  ZNEG  = 2,
  YPOS  = 4,
  YNEG  = 8,
  XPOS  = 16,
  XNEG  = 32,
  NONE  = 64,
} wu_axis_t;

class KXTJ3
{
  public:
  KXTJ3(uint8_t);

  /*
  Accelerometer range = 2, 4, 8, 16g
  Sample Rate - 0.781, 1.563, 3.125, 6.25, 12.5, 25, 50, 100, 200, 400, 800,
  1600Hz Output Data Rates ≥400Hz will force device into High Resolution mode
  */
  kxtj3_status_t begin(float sampleRate, uint8_t accRange,
                       bool highResSet = false, bool debugSet = false);

  // Enables 14-bit operation mode for Accelerometer range 8g/16g
  kxtj3_status_t enable14Bit(uint8_t accRange);

  // readRegister reads one 8-bit register
  kxtj3_status_t readRegister(uint8_t *outputPointer, uint8_t offset);

  // Reads two 8-bit regs, LSByte then MSByte order, and concatenates them.
  // Acts as a 16-bit read operation
  kxtj3_status_t readRegisterInt16(int16_t *outputPointer, uint8_t offset);

  // Writes an 8-bit byte;
  kxtj3_status_t writeRegister(uint8_t offset, uint8_t dataToWrite);

  // Configure Interrupts
  // @Threshold from -2048 to 2047 counts
  // @moveDur   from 1 to 255 counts
  // @naDur			from 1 to 255 counts
  // @polarity changes active low/high of physical interrupt pin
  // @wuRate    from 0.781 to 100Hz; -1 uses IMU data rate instead
  // @latched sets whether to use latched or unlatched interrupt
  // @pulsed sets whether to pulse the interrupt pin when active
  // @motion sets whether to trigger interrupt with wake-up function
  // @dataReady sets whether to trigger interrupt when new data is ready
  // @intPin sets whether to enable the interrupt pin or not
  // Threshold (g) = threshold (counts) / 256(counts/g)
  // timeDur (sec) = WAKEUP_COUNTER (counts) / Wake-Up Function ODR(Hz)
  // Non-ActivityTime (sec) = NA_COUNTER (counts) / Wake-Up Function ODR(Hz)
  kxtj3_status_t intConf(int16_t threshold, uint8_t moveDur, uint8_t naDur,
                         bool polarity = HIGH, float wuRate = -1,
                         bool latched = false, bool pulsed = false,
                         bool motion = true, bool dataReady = false,
                         bool intPin = true);

  kxtj3_status_t intDisableAxis(wu_axis_t first);
  kxtj3_status_t intDisableAxis(wu_axis_t first, wu_axis_t second);
  kxtj3_status_t intDisableAxis(wu_axis_t first, wu_axis_t second,
                                wu_axis_t third);
  kxtj3_status_t intDisableAxis(wu_axis_t first, wu_axis_t second,
                                wu_axis_t third, wu_axis_t fourth);
  kxtj3_status_t intDisableAxis(wu_axis_t first, wu_axis_t second,
                                wu_axis_t third, wu_axis_t fourth,
                                wu_axis_t fifth);

  // Checks to see if new data is ready (only works if DRDY interrupt enabled)
  bool dataReady(void);

  // Checks if the reason for the interrupt is the Wake-Up Function if enabled
  bool motionDetected(void);

  // Returns the direction that caused the Wake-Up Function to trigger
  wu_axis_t motionDirection(void);

  // Resets the interrupt latch
  kxtj3_status_t resetInterrupt(void);

  // Read axis acceleration as Float
  float axisAccel(axis_t _axis);

  // Set IMU to Standby ~0.9uA, also Enable configuration -> PC1 bit in
  // CTRL_REG1 must first be set to “0”
  kxtj3_status_t standby(bool _en = true);

  private:
  bool highRes   = false;
  bool debugMode = false;
  bool en14Bit   = false;
  uint8_t I2CAddress;
  float accelSampleRate; // Sample Rate - 0.781, 1.563, 3.125, 6.25, 12.5, 25,
                         // 50, 100, 200, 400, 800, 1600Hz
  uint8_t accelRange;    // Accelerometer range = 2, 4, 8, 16g

  // Apply settings at .begin()
  kxtj3_status_t applySettings(void);

  // Performs software reset
  kxtj3_status_t softwareReset(void);

  // ReadRegisterRegion takes a uint8 array address as input and reads
  //   a chunk of memory into that array.
  kxtj3_status_t readRegisterRegion(uint8_t *, uint8_t, uint8_t);

  // Start-up delay for coming out of standby
  void startupDelay(void);
};

// Device Registers
#define KXTJ3_WHO_AM_I 0x0F
#define KXTJ3_DCST_RESP                                                        \
  0x0C // used to verify proper integrated circuit functionality.
       // It always has a byte value of 0x55
#define KXTJ3_SOFT_REST          0x7F // used during software reset
#define KXTJ3_XOUT_L             0x06
#define KXTJ3_XOUT_H             0x07
#define KXTJ3_YOUT_L             0x08
#define KXTJ3_YOUT_H             0x09
#define KXTJ3_ZOUT_L             0x0A
#define KXTJ3_ZOUT_H             0x0B

#define KXTJ3_STATUS_REG         0x18
#define KXTJ3_INT_SOURCE1        0x16
#define KXTJ3_INT_SOURCE2        0x17
#define KXTJ3_INT_REL            0x1A

#define KXTJ3_CTRL_REG1          0x1B // *
#define KXTJ3_CTRL_REG2          0x1D // *

#define KXTJ3_INT_CTRL_REG1      0x1E // *
#define KXTJ3_INT_CTRL_REG2      0x1F // *

#define KXTJ3_DATA_CTRL_REG      0x21 // *
#define KXTJ3_WAKEUP_COUNTER     0x29 // *
#define KXTJ3_NA_COUNTER         0x2A // *
#define KXTJ3_SELF_TEST          0x3A // *

#define KXTJ3_WAKEUP_THRESHOLD_H 0x6A // *
#define KXTJ3_WAKEUP_THRESHOLD_L 0x6B // *

// * Note that to properly change the value of this register, the PC1 bit in
// CTRL_REG1 must first be set to “0”.

#endif // End of __KXTJ3_IMU_H__ definition check
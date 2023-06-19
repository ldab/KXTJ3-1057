/****************************************************************
KXTJ3-1057 for Arduino
Latched Interrupt & New Data Ready Example Sketch
Leonardo Bispo & Nomake Wan
June, 2023
https://github.com/ldab/KXTJ3-1057
Resources:
Uses Wire.h for I2C operation

Distributed as-is; no warranty is given.
****************************************************************/

// Include the KXTJ3-1057 library (includes Wire.h)
#include "kxtj3-1057.h"

float sampleRate =
   0.781; // HZ - Samples per second - 0.781, 1.563, 3.125, 6.25,
          // 12.5, 25, 50, 100, 200, 400, 800, 1600Hz
          // Sample rates ≥ 400Hz force High Resolution mode on
uint8_t accelRange = 2; // 14-bit Mode only supports 8g or 16g
bool highRes = true; // Set high resolution mode on
int16_t threshold = 0; // Sets wake-up threshold to default
uint8_t moveDur = 0; // Sets movement duration to default
uint8_t naDur = 0; // Sets non-activity duration to default
bool polarity = HIGH; // Sets INT pin polarity to active HIGH
float wuRate = -1; // Sets wake-up sample rate to IMU sample rate
bool latched = true; // Enables latched interrupt mode
bool pulsed = false; // Disables pulsed interrupt mode
bool motion = false; // Disables Motion Detection interrupt
bool dataReady = true; // Enables New Data Ready interrupt
bool intPin = false; // Disables INT pin operation

KXTJ3 myIMU(0x0E); // Address can be 0x0E or 0x0F

void setup()
{
  Serial.begin(115200);
  delay(1000); // wait until serial is open...

  if (myIMU.begin(sampleRate, accelRange, highRes) ==
  IMU_SUCCESS) {
    Serial.println("IMU initialized.");
  } else {
    Serial.println("Failed to initialize IMU.");
    while(true); // stop running sketch if failed
  }

  uint8_t readData = 0;

  // Get the ID:
  if (myIMU.readRegister(&readData, KXTJ3_WHO_AM_I) ==
  IMU_SUCCESS) {
  Serial.print("Who am I? 0x");
  Serial.println(readData, HEX);
  } else {
    Serial.println("Communication error, stopping.");
    while(true); // stop running sketch if failed
  }

  // Sets all Wake-Up parameters to defaults
  // Disables Wake-Up and INT pin functions
  // Enables Latched mode and Data Ready mode
  if (myIMU.intConf(threshold, moveDur, naDur, polarity, wuRate,
  latched, pulsed, motion, dataReady, intPin) == IMU_SUCCESS)
  {
    Serial.println("Latched interrupt configured.");
  } else {
    Serial.println("Communication error, stopping.");
    while(true); // stop running sketch if failed
  }
}

void loop()
{
  // Check to see if new data is ready
  if (myIMU.dataReady())
  {
    // Since new data is ready, read it
    Serial.print(" Acceleration X float = ");
    Serial.println(myIMU.axisAccel(X), 4);
    Serial.print(" Acceleration Y float = ");
    Serial.println(myIMU.axisAccel(Y), 4);
    Serial.print(" Acceleration Z float = ");
    Serial.println(myIMU.axisAccel(Z), 4);

    // Reading acceleration data resets the Data Ready latch
    // Motion Detection requires manually resetting the latch
    //myIMU.ResetInterrupt();
  }
}

/****************************************************************
kXTJ3-1057 Arduino
Leonardo Bispo
May, 2020
https://github.com/ldab/kxtj3-1057
Resources:
Uses Wire.h for i2c operation

Distributed as-is; no warranty is given.
****************************************************************/

// Include the KXTJ3-1057 library (includes Wire.h)
#include "kxtj3-1057.h"

float sampleRate =
    6.25; // HZ - Samples per second - 0.781, 1.563, 3.125, 6.25,
          // 12.5, 25, 50, 100, 200, 400, 800, 1600Hz
          // Sample rates ≥ 400Hz force High Resolution mode on
uint8_t accelRange = 2; // Accelerometer range = 2, 4, 8, 16g
bool highRes = false; // High Resolution mode on/off

KXTJ3 myIMU(0x0E); // Address can be 0x0E or 0x0F

void setup()
{
  // put your setup code here, to run once:

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
  myIMU.readRegister(&readData, KXTJ3_WHO_AM_I);
  Serial.print("Who am I? 0x");
  Serial.println(readData, HEX);
}

void loop()
{
  // Take IMU out of standby
  myIMU.standby(false);

  int16_t dataHighres = 0;

  if (myIMU.readRegisterInt16(&dataHighres, KXTJ3_OUT_X_L) ==
  IMU_SUCCESS) {
    Serial.print(" Acceleration X RAW = ");
    Serial.println(dataHighres);

    // Read accelerometer data in mg as Float
    Serial.print(" Acceleration X float = ");
    Serial.println(myIMU.axisAccel(X), 4);
  }

  if (myIMU.readRegisterInt16(&dataHighres, KXTJ3_OUT_Y_L) ==
  IMU_SUCCESS) {
    Serial.print(" Acceleration Y RAW = ");
    Serial.println(dataHighres);

    // Read accelerometer data in mg as Float
    Serial.print(" Acceleration Y float = ");
    Serial.println(myIMU.axisAccel(Y), 4);
  }

  if (myIMU.readRegisterInt16(&dataHighres, KXTJ3_OUT_Z_L) ==
  IMU_SUCCESS) {
    Serial.print(" Acceleration Z RAW = ");
    Serial.println(dataHighres);

    // Read accelerometer data in mg as Float
    Serial.print(" Acceleration Z float = ");
    Serial.println(myIMU.axisAccel(Z), 4);
  }

  // Put IMU back into standby
  myIMU.standby(true);

  // Delay so serial data is human readable
  delay(1000);
}

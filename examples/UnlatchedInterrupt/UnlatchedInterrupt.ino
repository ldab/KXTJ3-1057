/****************************************************************
KXTJ3-1057 for Arduino
Unlatched Interrupt Example Sketch
Leonardo Bispo & Nomake Wan
June, 2023
https://github.com/ldab/KXTJ3-1057
Resources:
Uses Wire.h for i2c operation

Distributed as-is; no warranty is given.
****************************************************************/

// Include the KXTJ3-1057 library (includes Wire.h)
#include "kxtj3-1057.h"

float sampleRate =
    6.25; // HZ - Samples per second - 0.781, 1.563, 3.125, 6.25,
          // 12.5, 25, 50, 100, 200, 400, 800, 1600Hz
uint8_t accelRange = 2; // 14-bit Mode only supports 8g or 16g
bool highRes = true; // Set high resolution mode on
bool detectedInterrupt = false;

KXTJ3 myIMU(0x0E); // Address can be 0x0E or 0x0F

void setup()
{
  Serial.begin(115200);

  // This pin will be used to detect the interrupt trigger
  // Connect the INT pin of the IMU to this pin
  pinMode(1, INPUT); // Pin D1

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

  // 128 / 256 = 0.5g change in acceleration threshold
  // 1 / 6.25 = 160ms movement duration
  // 10 / 6.25 = 1.6s non-activity reset
  // INT pin active HIGH
  // Wake-Up Sample Rate = IMU Sample Rate (6.25 Hz)
  myIMU.intConf(128, 1, 10, HIGH);
}

void loop()
{
  // This reads the state of Pin D1 to see if it's HIGH
  // If it is and we haven't already detected interrupt, print!
  // If it is LOW we reset the sentinel value to try again
  if (digitalRead(1) == HIGH && !detectedInterrupt)
  {
    Serial.println("Interrupt fired!");
    detectedInterrupt = true;
  }
  else if (digitalRead(1) == LOW && detectedInterrupt)
  {
    detectedInterrupt = false;
  }
}

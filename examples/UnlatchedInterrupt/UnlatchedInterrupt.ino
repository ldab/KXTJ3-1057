/****************************************************************
KXTJ3-1057 for Arduino
Unlatched Interrupt Example Sketch
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
    6.25; // HZ - Samples per second - 0.781, 1.563, 3.125, 6.25,
          // 12.5, 25, 50, 100, 200, 400, 800, 1600Hz
          // Sample rates ≥ 400Hz force High Resolution mode on
uint8_t accelRange = 2; // 14-bit Mode only supports 8g or 16g
bool highRes = true; // Set high resolution mode on
bool detectedInterrupt = false; // Create variable for detection
int16_t threshold = 128; // Sets wake-up threshold to 0.5g
uint8_t moveDur = 1; // Sets movement duration to 160ms
uint8_t naDur = 10; // Sets non-activity duration to 1.6s
bool polarity = HIGH; // Sets INT pin polarity to active HIGH

KXTJ3 myIMU(0x0E); // Address can be 0x0E or 0x0F

// Function to print user-friendly interrupt axis names
String printAxis(wu_axis_t axis)
{
  switch (axis)
  {
    case ZPOS:
      return "ZPOS";
      break;
    case ZNEG:
      return "ZNEG";
      break;
    case YPOS:
      return "YPOS";
      break;
    case YNEG:
      return "YNEG";
      break;
    case XPOS:
      return "XPOS";
      break;
    case XNEG:
      return "XNEG";
      break;
    default:
      return "NONE";
      break;
  }
}

void setup()
{
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
  if (myIMU.readRegister(&readData, KXTJ3_WHO_AM_I) ==
  IMU_SUCCESS) {
    Serial.print("Who am I? 0x");
    Serial.println(readData, HEX);
  } else {
    Serial.println("Communication error, stopping.");
    while(true); // stop running sketch if failed
  }

  // 128 / 256 = 0.5g change in acceleration threshold
  // 1 / 6.25 = 160ms movement duration
  // 10 / 6.25 = 1.6s non-activity reset
  // INT pin active HIGH
  // Wake-Up Sample Rate == IMU Sample Rate (6.25 Hz)
  if (myIMU.intConf(threshold, moveDur, naDur, polarity) ==
  IMU_SUCCESS) {
    Serial.println("Unlatched interrupt configured.");
  } else {
    Serial.println("Communication error, stopping.");
    while(true); // stop running sketch if failed
  }

  // Disable the Z-axis for motion detection
  if (myIMU.intDisableAxis(ZPOS, ZNEG) == IMU_SUCCESS)
  {
    Serial.println("Disabled Z-axis from interrupt.");
  } else {
    Serial.println("Communication error, stopping.");
    while(true); // stop running sketch if failed
  }
}

void loop()
{
  // This reads the state of Pin D1 to see if it's HIGH
  // If it is and we haven't already detected interrupt, print
  // Also asks which axis triggered the interrupt
  // If pin goes LOW we reset the sentinel value to try again
  if (digitalRead(1) == HIGH && !detectedInterrupt)
  {
    Serial.println(" Interrupt fired!");
    Serial.print(" Motion Direction: ");
    Serial.println(printAxis(myIMU.motionDirection()));
    myIMU.resetInterrupt(); // reset direction bit
    detectedInterrupt = true;
  }
  else if (digitalRead(1) == LOW && detectedInterrupt)
  {
    detectedInterrupt = false;
  }
}

/******************************************************************************
KXTJ3-1057.cpp
KXTJ3-1057 Arduino
Leonardo Bispo
May, 2020
https://github.com/ldab/KXTJ3-1057
Resources:
Uses Wire.h for i2c operation

Distributed as-is; no warranty is given.
******************************************************************************/

#include "KXTJ3-1057.h"
#include "stdint.h"

#include "Wire.h"

//****************************************************************************//
//
//  Default construction is I2C mode, address 0x19.
//
//****************************************************************************//
KXTJ3::KXTJ3( uint8_t inputArg = 0x0E )
{
  I2CAddress = inputArg;
}

kxtj3_status_t KXTJ3::begin( float SampleRate, uint8_t accRange )
{

	_DEBBUG("Configuring IMU");

	kxtj3_status_t returnError = IMU_SUCCESS;

  Wire.begin();
  
	// Start-up time, Figure 1: Typical StartUp Time - DataSheet
	#ifdef HIGH_RESOLUTION
		if			( SampleRate < 1)		delay(1300);
		else if ( SampleRate < 3)		delay(650);
		else if ( SampleRate < 6)		delay(350);
		else if ( SampleRate < 25)	delay(180);
		else												delay(45);
	#else
		delay(2);
	#endif

	//Check the ID register to determine if the operation was a success.
	uint8_t _whoAmI;
	
	readRegister(&_whoAmI, KXTJ3_WHO_AM_I);

	if( _whoAmI != 0x35 )
	{
		returnError = IMU_HW_ERROR;
	}

	accelSampleRate = SampleRate;
	accelRange 			= accRange;

	_DEBBUG("Apply settings");
	applySettings();

	return returnError;
}

//****************************************************************************//
//
//  ReadRegisterRegion
//
//  Parameters:
//    *outputPointer -- Pass &variable (base address of) to save read data to
//    offset -- register to read
//    length -- number of bytes to read
//
//****************************************************************************//
kxtj3_status_t KXTJ3::readRegisterRegion(uint8_t *outputPointer , uint8_t offset, uint8_t length)
{
	kxtj3_status_t returnError = IMU_SUCCESS;

	//define pointer that will point to the external space
	uint8_t i = 0;
	uint8_t c = 0;

  Wire.beginTransmission(I2CAddress);
  offset |= 0x80; //turn auto-increment bit on, bit 7 for I2C
  Wire.write(offset);
  if( Wire.endTransmission() != 0 )
  {
    returnError = IMU_HW_ERROR;
  }
  else  //OK, all worked, keep going
  {
    // request 6 bytes from slave device
    Wire.requestFrom(I2CAddress, length);
    while ( (Wire.available()) && (i < length))  // slave may send less than requested
    {
      c = Wire.read(); // receive a byte as character
      *outputPointer = c;
      outputPointer++;
      i++;
    }
  }

	return returnError;
}

//****************************************************************************//
//
//  ReadRegister
//
//  Parameters:
//    *outputPointer -- Pass &variable (address of) to save read data to
//    offset -- register to read
//
//****************************************************************************//
kxtj3_status_t KXTJ3::readRegister(uint8_t* outputPointer, uint8_t offset) {
	//Return value
	uint8_t result = 0;
	uint8_t numBytes = 1;
	kxtj3_status_t returnError = IMU_SUCCESS;

	Wire.beginTransmission(I2CAddress);
	Wire.write(offset);
	
	if( Wire.endTransmission() != 0 )
	{
		returnError = IMU_HW_ERROR;
	}

	Wire.requestFrom(I2CAddress, numBytes);

	while ( Wire.available() ) // slave may send less than requested
	{
		result = Wire.read(); 	 // receive a byte as a proper uint8_t
	}

	_DEBBUG("Read register 0x", offset, " = ", result);

	*outputPointer = result;
	return returnError;
}

//****************************************************************************//
//
//  readRegisterInt16
//
//  Parameters:
//    *outputPointer -- Pass &variable (base address of) to save read data to
//    offset -- register to read
//
//****************************************************************************//
kxtj3_status_t KXTJ3::readRegisterInt16( int16_t* outputPointer, uint8_t offset )
{
	//offset |= 0x80; //turn auto-increment bit on
	uint8_t myBuffer[2];
	kxtj3_status_t returnError = readRegisterRegion(myBuffer, offset, 2);  //Does memory transfer
	int16_t output = (int16_t)myBuffer[0] | int16_t(myBuffer[1] << 8);

	_DEBBUG("12 bit from 0x", offset, " = ", output);
	*outputPointer = output;
	return returnError;
}

//****************************************************************************//
//
//  writeRegister
//
//  Parameters:
//    offset -- register to write
//    dataToWrite -- 8 bit data to write to register
//
//****************************************************************************//
kxtj3_status_t KXTJ3::writeRegister(uint8_t offset, uint8_t dataToWrite) {
	kxtj3_status_t returnError = IMU_SUCCESS;

  //Write the byte
  Wire.beginTransmission(I2CAddress);
  Wire.write(offset);
  Wire.write(dataToWrite);
  if( Wire.endTransmission() != 0 )
  {
    returnError = IMU_HW_ERROR;
  }

	return returnError;
}

// Read axis acceleration as Float
float KXTJ3::axisAccel( axis_t _axis)
{
	int16_t outRAW;
	uint8_t regToRead = 0;
	switch (_axis)
	{
		case 0:
			// X axis
			regToRead = KXTJ3_OUT_X_L;
			break;
		case 1:
			// Y axis
			regToRead = KXTJ3_OUT_Y_L;
			break;
		case 2:
			// Z axis
			regToRead = KXTJ3_OUT_Z_L;
			break;
	
		default:
			// Not valid axis return NAN
			return NAN;
			break;
	}

	readRegisterInt16( &outRAW, regToRead );

	float outFloat;

	switch( accelRange )
	{
		case 2:
		outFloat = (float)outRAW / 15987;
		break;
		case 4:
		outFloat = (float)outRAW / 7840;
		break;
		case 8:
		outFloat = (float)outRAW / 3883;
		break;
		case 16:
		outFloat = (float)outRAW / 1280;
		break;
		default:
		outFloat = 0;
		break;
	}

	return outFloat;

}

	// Set the IMU to Power-down mode ~ 0.5uA;
kxtj3_status_t KXTJ3::imu_power_down( void )
{
	// ODR[3:0] -> (0000: power-down mode; others: Refer to Table 31: Data rate configuration)
	return writeRegister(KXTJ3_CTRL_REG1, 0x00);
}

kxtj3_status_t	KXTJ3::enable_config( void )
{
	uint8_t _pc1 = 0x7E;
	uint8_t _ctrl;

	// PC1 bit -> controls the operating mode of the KXTJ3 and allow modificaitons
	readRegister(&_ctrl, KXTJ3_CTRL_REG1);
	
	_ctrl &= _pc1;

	return writeRegister(KXTJ3_CTRL_REG1, _ctrl);
}

//****************************************************************************//
//
//  Apply settings passed to .begin();
//
//****************************************************************************//
void KXTJ3::applySettings( void )
{
	uint8_t dataToWrite = 0;  //Temporary variable

	enable_config();
	
	//Build DATA_CTRL_REG

	//  Convert ODR
	if(accelSampleRate < 1)					dataToWrite |= 0x08;	// 0.781Hz
	else if(accelSampleRate < 2)		dataToWrite |= 0x09;	// 1.563Hz
	else if(accelSampleRate < 4)		dataToWrite |= 0x0A;	// 3.125Hz
	else if(accelSampleRate < 8)		dataToWrite |= 0x0B;	// 6.25Hz
	else if(accelSampleRate < 16)		dataToWrite |= 0x00;	// 12.5Hz
	else if(accelSampleRate < 30)		dataToWrite |= 0x01;	// 25Hz
	else if(accelSampleRate < 60)		dataToWrite |= 0x02;	// 50Hz
	else if(accelSampleRate < 150)	dataToWrite |= 0x03;	// 100Hz
	else if(accelSampleRate < 250)	dataToWrite |= 0x04;	// 200Hz
	else if(accelSampleRate < 450)	dataToWrite |= 0x05;	// 400Hz
	else if(accelSampleRate < 850)	dataToWrite |= 0x06;	// 800Hz
	else														dataToWrite	|= 0x07;	// 1600Hz

	//Now, write the patched together data
	_DEBBUG ("KXTJ3_DATA_CTRL_REG: 0x", dataToWrite);
	writeRegister(KXTJ3_DATA_CTRL_REG, dataToWrite);

	//Build CTRL_REG1
	
	// LOW power, 8-bit mode
	dataToWrite = 0x80;

	#ifdef HIGH_RESOLUTION
		dataToWrite = 0xC0;
	#endif

	//  Convert scaling
	switch(accelRange)
	{	
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

	//Now, write the patched together data
	_DEBBUG ("KXTJ3_CTRL_REG1: 0x", dataToWrite);
	writeRegister(KXTJ3_CTRL_REG1, dataToWrite);

}

//****************************************************************************//
//
//  Configure interrupts 1 or 2, stop or move, threshold and duration
//	Durationsteps and maximum values depend on the ODR chosen.
//
//****************************************************************************//
kxtj3_status_t KXTJ3::intConf(event_t moveType, 
						uint8_t threshold,
						uint8_t timeDur,
						bool		polarity )
{
	// Note that to properly change the value of this register, the PC1 bit in CTRL_REG1must first be set to “0”.
	enable_config();

	kxtj3_status_t returnError = IMU_SUCCESS;

	// Build INT_CTRL_REG1

	uint8_t dataToWrite = 0x2A;  // Interrupt enabled, active low, non-latched
	
	if( polarity == HIGH )
		dataToWrite |= (0x10);		// Active HIGH

	_DEBBUG ("KXTJ3_INT_CTRL_REG1: 0x", dataToWrite);
	returnError = writeRegister(KXTJ3_INT_CTRL_REG1, dataToWrite);
	
	// Build INT_CTRL_REG2

	dataToWrite = 0x3F;  // enable interrupt on all axis any direction

	_DEBBUG ("KXTJ3_INT_CTRL_REG1: 0x", dataToWrite);
	returnError = writeRegister(KXTJ3_INT_CTRL_REG2, dataToWrite);
	
	return returnError;
}

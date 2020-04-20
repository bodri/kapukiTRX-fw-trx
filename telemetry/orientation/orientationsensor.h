/*!
 *  @file Adafruit_BNO055.h
 *
 *  This is a library for the BNO055 orientation sensor
 *
 *  Designed specifically to work with the Adafruit BNO055 Breakout.
 *
 *  Pick one up today in the adafruit shop!
 *  ------> https://www.adafruit.com/product/2472
 *
 *  These sensors use I2C to communicate, 2 pins are required to interface.
 *
 *  Adafruit invests time and resources providing this open source code,
 *  please support Adafruit andopen-source hardware by purchasing products
 *  from Adafruit!
 *
 *  K.Townsend (Adafruit Industries)
 *
 *  MIT license, all text above must be included in any redistribution
 */

#ifndef __O_H__
#define __O_H__


#include "bno055.h"
#include "imumaths.h"

/** BNO055 Address A **/
#define BNO055_ADDRESS_A (0x28)
/** BNO055 Address B **/
#define BNO055_ADDRESS_B (0x29)
/** BNO055 ID **/
#define BNO055_ID (0xA0)

/*!
 *  @brief  Class that stores state and functions for interacting with
 *          BNO055 Sensor
 */
class OrientationSensor {
public:
	OrientationSensor(uint8_t address = BNO055_ADDRESS_A);

	bool begin();

	bno055_quaternion_t getQuaternion();
	int8_t getTemperature();


	/* Power managments functions */
//	void enterSuspendMode();
//	void enterNormalMode();

private:
	uint8_t i2cAddress;
	struct bno055_t sensorDevice;
};

#endif

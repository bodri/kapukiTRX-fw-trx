/**
 * @file orientationsensor.h
 * @brief Wrapper class for BNO055 orientation sensor.
 *
 * @author Varadi, Gyorgy, aka bodri
 * Contact: bodri@bodrico.com
 *
 * @bug No known bugs.
 *
 * MIT license, all text above must be included in any redistribution
 *
 */

#ifndef __ORIENTATIONSENSOR_H__
#define __ORIENTATIONSENSOR_H__


#include "bno055.h"
#include "imumaths.h"

class OrientationSensor {
public:
	OrientationSensor(uint8_t address = BNO055_I2C_ADDR1) :
		i2cAddress(i2cAddress << 1)
	{ }

	bool start();

	bno055_quaternion_t getQuaternion();
	int8_t getTemperature();

	void enterSuspendMode();
	void enterNormalMode();

private:
	uint8_t i2cAddress;
	struct bno055_t sensorDevice;
};

#endif //__ORIENTATIONSENSOR_H__

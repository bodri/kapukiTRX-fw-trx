/**
 * @file altitudesensor.cpp
 * @brief Wrapper class for BMP388 pressure sensor.
 *
 * @author Varadi, Gyorgy, aka bodri
 * Contact: bodri@bodrico.com
 *
 * @bug No known bugs.
 *
 * MIT license, all text above must be included in any redistribution
 *
 */

#ifndef __ALTITUDESENSOR_H__
#define __ALTITUDESENSOR_H__

#include "bmp3.h"

class AltitudeSensor {
public:
	AltitudeSensor(uint8_t i2cAddress = BMP3_I2C_ADDR_PRIM) :
		i2cAddress(i2cAddress << 1)
	{ };

	bool start();

	double readTemperature(void);
	double readPressure(void);
	double readAltitude(double seaLevel);

	bool performReading(void);

	double temperature;
	double pressure;

private:
	uint8_t i2cAddress;
	struct bmp3_dev sensorDevice;
};

#endif //__ALTITUDESENSOR_H__

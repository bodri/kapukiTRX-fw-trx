/*!
 * @file Adafruit_BMP3XX.h
 *
 * Adafruit BMP3XX temperature & barometric pressure sensor driver
 *
 * This is the documentation for Adafruit's BMP3XX driver for the
 * Arduino platform.  It is designed specifically to work with the
 * Adafruit BMP388 breakout: https://www.adafruit.com/products/3966
 *
 * These sensors use I2C or SPI to communicate
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Written by Ladyada for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */

#ifndef __BMP3XX_H__
#define __BMP3XX_H__

#include "bmp3.h"

/** Adafruit_BMP3XX Class for both I2C and SPI usage.
 *  Wraps the Bosch library for Arduino usage
 */

class BMP388 {
public:
	BMP388();

	bool begin(uint8_t addr = BMP3_I2C_ADDR_SEC);
	float readTemperature(void);
	float readPressure(void);
	float readAltitude(float seaLevel);

	bool setTemperatureOversampling(uint8_t os);
	bool setPressureOversampling(uint8_t os);
	bool setIIRFilterCoeff(uint8_t fs);
	bool setOutputDataRate(uint8_t odr);

	/// Perform a reading in blocking mode
	bool performReading(void);

	/// Temperature (Celsius) assigned after calling performReading()
	double temperature;
	/// Pressure (Pascals) assigned after calling performReading()
	double pressure;

private:
	bool filterEnabled;
	bool temperatureOversamplingEnabled;
	bool pressureOversamplingEnabled;
	bool outputDataRateEnabled;
	uint8_t i2cAddress;

	struct bmp3_dev sensorDevice;
};

#endif

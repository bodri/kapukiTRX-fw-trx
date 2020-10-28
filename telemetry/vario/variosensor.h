/**
 * @file variosensor.cpp
 * @brief Wrapper class for BMP388 pressure sensor.
 *
 * @author Varadi Gyorgy aka bodri
 * Contact: bodri@bodrico.com
 *
 * @bug No known bugs.
 *
 * MIT license, all text above must be included in any redistribution
 *
 */

#ifndef __VARIOSENSOR_H__
#define __VARIOSENSOR_H__

#include <vario/bmp3.h>
#include "sensor.h"

#include <memory>

class VarioSensor : public Sensor {
public:
	VarioSensor(uint8_t i2cAddress = BMP3_I2C_ADDR_PRIM);

	enum SensorData {
		sensor = 0,
		temperature,
		pressure,
		verticalSpeed,
		altitude
	};

	virtual bool start() override;
	virtual size_t dataSize() override;
	virtual std::string getDescription() override;
	virtual std::string getData() override;

private:
	uint8_t i2cAddress;
	struct bmp3_dev sensorDevice;

	size_t telemetryDataSize { 0 };

	std::shared_ptr<bmp3_data> performReading(void);
};

#endif //__VARIOSENSOR_H__

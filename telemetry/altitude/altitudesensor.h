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
#include "sensor.h"

#include <memory>

class AltitudeSensor : public Sensor {
public:
	AltitudeSensor(uint8_t i2cAddress = BMP3_I2C_ADDR_PRIM);

	virtual bool start() override;
	virtual size_t dataSize() override;
	virtual std::string getDescription() override;
	virtual std::string getData() override;

private:
	uint8_t i2cAddress;
	struct bmp3_dev sensorDevice;

	TelemetryData *temperature;
	TelemetryData *pressure;
	size_t telemetryDataSize { 0 };

	std::shared_ptr<bmp3_data> performReading(void);
};

#endif //__ALTITUDESENSOR_H__

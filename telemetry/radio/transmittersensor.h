/**
 * @file transmittersensor.h
 * @brief Sensor for transmitter internal telemetry data.
 *
 * Sensor data:
 * 			Name			type		decimal point
 * 			------------------------------------------
 *
 * @author Varadi, Gyorgy aka bodri
 * Contact: bodri@bodrico.com
 *
 * @bug No known bugs.
 *
 * MIT license, all text above must be included in any redistribution
 *
 */

#ifndef __TRANSMITTERSENSOR_H__
#define __TRANSMITTERSENSOR_H__

#include "sensor.h"

#include <stdint.h>

class TransmitterSensor : public Sensor {
public:
	TransmitterSensor();

	enum SensorData {
		sensor = 0,
		rssi1,
		rssi2,
		linkQuality
	};

	virtual bool start() override;
	virtual size_t dataSize() override;
	virtual std::string getDescription() override;
	virtual std::string getData() override;

private:
	size_t telemetryDataSize { 0 };
};



#endif /* __TRANSMITTERSENSOR_H__ */

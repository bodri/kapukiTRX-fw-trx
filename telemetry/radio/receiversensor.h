/**
 * @file receiversensor.h
 * @brief Sensor for receiver telemetry data.
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

#ifndef __RECEIVERSENSOR_H__
#define __RECEIVERSENSOR_H__


#include "sensor.h"

#include <stdint.h>

class ReceiverSensor : public Sensor {
public:
	ReceiverSensor();

	enum SensorData {
		sensor = 0,
		receiverVoltage,
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

#endif //__RECEIVERSENSOR_H__

/**
 * @file orientationsensor.h
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

	virtual bool start() override;
	virtual size_t dataSize() override;
	virtual std::string getDescription() override;
	virtual std::string getData() override;

private:
	TelemetryData *receiverVoltage;
	TelemetryData *rssiModule1;
	TelemetryData *rssiModule2;
	size_t telemetryDataSize { 0 };
};

#endif //__RECEIVERSENSOR_H__

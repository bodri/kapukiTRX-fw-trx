/*
 * remotesensor.h
 *
 *  Created on: 15 May 2020
 *      Author: gvaradi
 */

#ifndef __REMOTESENSOR_H__
#define __REMOTESENSOR_H__

#include "sensor.h"

class RemoteSensor : public Sensor {
public:
	SensorInfo sensorInfo;

	RemoteSensor(uint16_t sensorInitInfo) { sensorInfo = sensorInitInfo; };

	bool start() override { return true; };
	size_t dataSize() override { return 0; };
	std::string getDescription() override { return ""; };
	std::string getData() override { return ""; };

private:
};



#endif /* __REMOTESENSOR_H__ */

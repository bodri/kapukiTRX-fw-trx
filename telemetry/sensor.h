/**
 * @file sensor.h
 * @brief Telemetry sensor protocol.
 *
 * @author Varadi Gyorgy aka bodri
 * Contact: bodri@bodrico.com
 *
 * @bug No known bugs.
 *
 * MIT license, all text above must be included in any redistribution
 *
 */

#ifndef __SENSOR_H__
#define __SENSOR_H__

#include "main.h"
#include "telemetrydata.h"

#include <string>
#include <vector>
#include <algorithm>

enum SensorIdentifier {
	unknown = 0,
	transmitterSensor,
	receiverSensor,
	orientationSensor,
	varioSensor,
	gpsSensor
};

struct SensorInfo {
	uint16_t identifier : 13;
	uint16_t numberOfTelemetryData : 3;

	SensorInfo &operator = (const uint16_t data) {
		identifier = (data >> 3) & 0x1FFF;
		numberOfTelemetryData = data & 0x7;
		return *this;
	};

	operator uint16_t() const {
		return (identifier << 3) | numberOfTelemetryData;
	}
} __attribute__ ((__packed__));

class Sensor {
public:
	SensorInfo sensorInfo;
	std::vector<TelemetryData *> telemetryDataArray;

	void processHeartBeat() { tick++; }

	TelemetryData *getTelemetryDataAt(uint8_t position) {
		std::vector<TelemetryData*>::iterator it = std::find_if(
				telemetryDataArray.begin(),
				telemetryDataArray.end(), [&](const TelemetryData *data) {
					return data->header.position == position;
				});
		if (it != telemetryDataArray.end()) {
			return *it;
		} else {
			return nullptr;
		}
	};

	virtual ~Sensor() = default;

	virtual bool start() = 0;
	virtual std::string getDescription() = 0;
	virtual size_t dataSize() = 0;
	virtual std::string getData() = 0;

private:
	uint8_t tick { 0 }; // 1 tick = 10 ms
	uint16_t refreshRateInMilliSeconds { 50 };
};

#endif /* __SENSOR_H__ */

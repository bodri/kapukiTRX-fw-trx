/**
 * @file telemetry.h
 * @brief Handles all telemetry sensors.
 *
 * @author Varadi Gyorgy aka bodri
 * Contact: bodri@bodrico.com
 *
 * @bug No known bugs.
 *
 * MIT license, all text above must be included in any redistribution
 *
 */

#ifndef __TELEMETRY_H__
#define __TELEMETRY_H__

#include "main.h"
#include "sensor.h"
#include "rflink.h"

#include <string>
#include <vector>

class Telemetry {
public:
	Telemetry() { };
	Telemetry(std::vector<Sensor *> sensors);

	void processHeartBeat();

	Telemetry &operator= (const Packet& packet);
	Sensor *getSensor(SensorIdentifier identifier);

	uint8_t prepareTelemetryPacket();
	void sendTelemetryPacket(Packet& packet);

private:
	std::vector<Sensor *> sensors { };
	std::string preparedTelemetryData { };

	Sensor *findOrCreateRemoteSensor(SensorInfo sensorInfo);
	TelemetryData *findOrCreateTelemetryData(Sensor *sensor, TelemetryDataHeader header);
};


#endif /* __TELEMETRY_H__ */

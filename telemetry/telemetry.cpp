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

#include "telemetry.h"

#include <cstring>

Telemetry::Telemetry(std::vector<Sensor *> sensors) :
		sensors(sensors) {
	for (auto &sensor : sensors) {
		sensor->start();
	}
}

void Telemetry::processHeartBeat() {
	for (auto &sensor : sensors) {
		sensor->processHeartBeat();
	}
}

void Telemetry::composeTelemetryPacket(Packet &packet) {
	std::string data;

	// TODO: size check, identifier, etc...
	for (auto &sensor : sensors) {
		if (sensor->dataSize() <= sizeof(packet.payload) - data.size()) {
			data.append(sensor->getData());
		}
	}

	strncpy((char *)packet.payload, data.c_str(), sizeof(packet.payload));
}



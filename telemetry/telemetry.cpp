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
#include "remotesensor.h"

#include <cstring>
#include <algorithm>
#include <memory>

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

Telemetry &Telemetry::operator= (const Packet& packet) {
	for (unsigned i = 0; i < sizeof(packet.payload);) {
		uint16_t sensorInfo = packet.payload[i] << 8 | packet.payload[i + 1];
		if (sensorInfo >> 4 == 0) {
			// no more sensors
			return *this;
		}

		Sensor *sensor = findOrCreateRemoteSensor(sensorInfo);

		i += 2;
		for (unsigned j = 0; j < sensor->sensorInfo.numberOfTelemetryData; j++) {
			uint8_t headerData = packet.payload[i++];
			uint8_t type = (headerData >> 2) & 0x3;

			TelemetryData *telemetryData = findOrCreateTelemetryData(sensor, headerData);

			int32_t value { 0 };
			switch (type) {
			case 3:
				value = packet.payload[i++] << 24;
			case 2:
				value |= packet.payload[i++] << 16;
			case 1:
				value |= packet.payload[i++] << 8;
			default:
				value |= packet.payload[i++];
				telemetryData->setValue(value);
			}
		}
	}

	return *this;
}

uint8_t Telemetry::prepareTelemetryPacket() {
	std::string data;
	size_t length { 0 };

	// TODO: size check, identifier, etc...
	for (auto sensor : sensors) {
		if (sensor->dataSize() <= sizeof(Packet::payload) - length) {
			length += sensor->dataSize();
			data.append(sensor->getData());
		}
	}

	preparedTelemetryData = data;
	return data.size();
}

void Telemetry::sendTelemetryPacket(Packet& packet) {
	memcpy(packet.payload, preparedTelemetryData.c_str(), packet.size);
}

//
// Private
//

Sensor *Telemetry::findOrCreateRemoteSensor(uint16_t sensorInfoData) {
	uint16_t sensorIdentifier = sensorInfoData >> 4;
	std::vector<Sensor*>::iterator it = std::find_if(sensors.begin(),
			sensors.end(), [&](const Sensor *elem) {
				return elem->sensorInfo.identifier == sensorIdentifier;
			});
	if (it != sensors.end()) {
		// sensor is already known
		return *it;
	} else {
		// no it's an unknown sensor, so create it
		Sensor *sensor = new RemoteSensor(sensorInfoData);
		sensors.push_back(sensor);
		return sensor;
	}
}

TelemetryData *Telemetry::findOrCreateTelemetryData(Sensor *sensor, uint8_t headerData) {
	uint8_t position = headerData >> 4;
	uint8_t type = (headerData >> 2) & 0x3;
	std::vector<TelemetryData*>::iterator it = std::find_if(
			sensor->telemetryDataArray.begin(),
			sensor->telemetryDataArray.end(), [&](const TelemetryData *data) {
				return data->header.position == position;
			});
	if (it != sensor->telemetryDataArray.end()) {
		// telemetry data is already known
		return *it;
	} else {
		// create new telemetry data
		TelemetryData *telemetryData = new TelemetryData(position, "", "", TelemetryDataType(type), 0);
		sensor->telemetryDataArray.push_back(telemetryData);
		return telemetryData;
	}
}

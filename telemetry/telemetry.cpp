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
#include "radio/receiversensor.h"
#include "orientation/orientationsensor.h"
#include "vario/variosensor.h"
#include "telemetrydata.h"

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

Sensor *Telemetry::getSensor(SensorIdentifier identifier) {
	std::vector<Sensor*>::iterator it = std::find_if(sensors.begin(),
			sensors.end(), [&](const Sensor *elem) {
				return elem->sensorInfo.identifier == identifier;
			});
	if (it != sensors.end()) {
		return *it;
	} else {
		return nullptr;
	}
}

Telemetry &Telemetry::operator= (const Packet& packet) {
	for (unsigned i = 0; i < sizeof(packet.payload);) {
		uint16_t sensorInfoData = (packet.payload[i] << 8) + packet.payload[i + 1];
		SensorInfo sensorInfo = *reinterpret_cast<SensorInfo *>(&sensorInfoData);
		if (sensorInfo.identifier == unknown) {
			// no more sensors
			return *this;
		}

		Sensor *sensor = findOrCreateRemoteSensor(sensorInfo);

		i += 2;
		for (unsigned j = 0; j < sensor->sensorInfo.numberOfTelemetryData; j++) {
			uint8_t headerData = packet.payload[i++];
			TelemetryDataHeader header = *reinterpret_cast<TelemetryDataHeader *>(&headerData);

			TelemetryData *telemetryData = findOrCreateTelemetryData(sensor, header);

			int32_t value { 0 };
			switch (header.type) {
			case int32_tdt:
				value = packet.payload[i++] << 24;
			case int24_tdt:
				value |= packet.payload[i++] << 16;
			case int16_tdt:
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

Sensor *Telemetry::findOrCreateRemoteSensor(SensorInfo sensorInfo) {
	Sensor *sensor = getSensor(static_cast<SensorIdentifier>(sensorInfo.identifier));
	if (sensor) {
		// sensor is already known
		return sensor;
	} else {
		// no it's an unknown sensor, so create it
		Sensor *sensor;
		switch (sensorInfo.identifier) {
		case SensorIdentifier::receiverSensor:
			sensor = dynamic_cast<Sensor*>(new ReceiverSensor());
			break;
		case SensorIdentifier::orientationSensor:
			sensor = dynamic_cast<Sensor*>(new OrientationSensor());
			break;
		case SensorIdentifier::varioSensor:
			sensor = dynamic_cast<Sensor*>(new VarioSensor());
			break;
		default:
			sensor = new RemoteSensor(sensorInfo);
		}

		sensors.push_back(sensor);
		return sensor;
	}
}

TelemetryData *Telemetry::findOrCreateTelemetryData(Sensor *sensor, TelemetryDataHeader header) {
	TelemetryData *telemetryData = sensor->getTelemetryDataAt(header.position);
	if (telemetryData) {
		// telemetry data is already known
		return telemetryData;
	} else {
		// create new telemetry data
		TelemetryData *telemetryData = new TelemetryData(header.position, "", "", header.type, 0);
		sensor->telemetryDataArray.push_back(telemetryData);
		return telemetryData;
	}
}

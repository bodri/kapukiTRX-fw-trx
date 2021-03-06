/**
 * @file receiversensor.cpp
 * @brief Sensor for receiver telemetry data.
 *
 * @author Varadi, Gyorgy aka bodri
 * Contact: bodri@bodrico.com
 *
 * @bug No known bugs.
 *
 * MIT license, all text above must be included in any redistribution
 *
 */

#include "receiversensor.h"

ReceiverSensor::ReceiverSensor() {
	this->telemetryDataArray = {
			new TelemetryData(SensorData::sensor, "Receiver", "", int8_tdt, 0),
			new TelemetryData(SensorData::receiverVoltage, "RX Voltage", "V", int8_tdt, 1),
			new TelemetryData(SensorData::rssi1, "RX RSSI #1", "dBm", int8_tdt, 0),
			new TelemetryData(SensorData::rssi2, "RX RSSI #2", "dBm", int8_tdt, 0),
			new TelemetryData(SensorData::linkQuality, "Link Quality", "%", int8_tdt, 0)
	};

	// Calculate size
	for (auto &telemetryData : telemetryDataArray) {
		telemetryDataSize += telemetryData->valueSize() + 1;
	}

	sensorInfo.identifier = receiverSensor;
	sensorInfo.numberOfTelemetryData = telemetryDataArray.size() - 1;
	telemetryDataSize += 1; // + sensorInfo - 1 for telemetryDataArray[0]
}

bool ReceiverSensor::start() {
    return true;
}

std::string ReceiverSensor::getDescription() {
	return "";
}

size_t ReceiverSensor::dataSize() {
	return telemetryDataSize;
}

std::string ReceiverSensor::getData() {
	std::string buffer;

	buffer.resize(2);
	buffer[0] = (uint16_t)sensorInfo >> 8;
	buffer[1] = (uint16_t)sensorInfo;
	for (auto &telemetryData : telemetryDataArray) {
		buffer.append(telemetryData->getValueRepresentation());
	}

	return buffer;
}

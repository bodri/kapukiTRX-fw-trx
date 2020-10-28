/**
 * @file transmittersensor.cpp
 * @brief Sensor for transmitter internal telemetry data.
 *
 * @author Varadi, Gyorgy aka bodri
 * Contact: bodri@bodrico.com
 *
 * @bug No known bugs.
 *
 * MIT license, all text above must be included in any redistribution
 *
 */

#include "transmittersensor.h"

TransmitterSensor::TransmitterSensor() {
	this->telemetryDataArray = {
			new TelemetryData(SensorData::sensor, "Transmitter", "", int8_tdt, 0),
			new TelemetryData(SensorData::rssi1, "TX RSSI #1", "dBm", int8_tdt, 0),
			new TelemetryData(SensorData::rssi2, "TX RSSI #2", "dBm", int8_tdt, 0),
			new TelemetryData(SensorData::linkQuality, "Link Quality", "%", int8_tdt, 0)
	};

	// Calculate size
	for (auto &telemetryData : telemetryDataArray) {
		telemetryDataSize += telemetryData->valueSize() + 1;
	}

	sensorInfo.identifier = transmitterSensor;
	sensorInfo.numberOfTelemetryData = telemetryDataArray.size() - 1;
	telemetryDataSize += 1; // + sensorInfo - 1 for telemetryDataArray[0]
}

bool TransmitterSensor::start() {
    return true;
}

std::string TransmitterSensor::getDescription() {
	return "";
}

size_t TransmitterSensor::dataSize() {
	return telemetryDataSize;
}

std::string TransmitterSensor::getData() {
	std::string buffer;

//		temperature->setValue((int8_t)data->temperature);
//		pressure->setValue((int32_t)data->pressure);

	buffer.resize(2);
	buffer[0] = (uint16_t)sensorInfo >> 8;
	buffer[1] = (uint16_t)sensorInfo;
	for (auto &telemetryData : telemetryDataArray) {
		buffer.append(telemetryData->getValueRepresentation());
	}

	return buffer;
}




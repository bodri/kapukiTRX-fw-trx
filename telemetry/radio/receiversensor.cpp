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
	TelemetryData *sensor = new TelemetryData(0, "Receiver", "", int8_tdt, 0);
	receiverVoltage = new TelemetryData(1, "Receiver Voltage", "V", int8_tdt, 1);
	rssiModule1 = new TelemetryData(1, "Transmitter RSSI #1", "dBm", int8_tdt, 0);
	rssiModule2 = new TelemetryData(2, "Transmitter RSSI #2", "dBm", int8_tdt, 0);

	this->telemetryDataArray = {
			sensor,
			receiverVoltage,
			rssiModule1,
			rssiModule2
	};

	// Calculate size
	for (auto &telemetryData : telemetryDataArray) {
		telemetryDataSize += telemetryData->valueSize() + 1;
	}

	sensorInfo.identifier = 0x2;
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

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

#include <string>
#include <vector>

struct TelemetryDataHeader {
	uint8_t position : 4;
	uint8_t type : 2;
	uint8_t decimalPointPosition : 2;

	TelemetryDataHeader &operator= (const uint8_t data) {
		position = data >> 4;
		type = (data >> 2) & 0x3;
		decimalPointPosition = data & 0x3;
		return *this;
	}

	operator uint8_t() const {
		return (position << 4) | (type << 2) | decimalPointPosition;
	}
};

typedef enum {
	int8_tdt = 0, // [-128, 127]
	int16_tdt, // [-32768, 32767]
	int24_tdt, // [-8388608, 8388607]
	int32_tdt, // [-2147483648, 2147483647]
} TelemetryDataType;

class TelemetryData {
public:
	TelemetryDataHeader header;

	TelemetryData(uint8_t position, std::string description, std::string unit, TelemetryDataType dataType, uint8_t decimalPointPosition) :
		description(description),
		unit(unit)
	{
		TelemetryDataHeader header;
		header.position = position & 0xF;
		header.type = dataType;
		header.decimalPointPosition = decimalPointPosition & 0x3;
		this->header = header;
	}

	void setValue(int32_t value) {
		this->value = value;
	}

	size_t valueSize() {
		return header.position == 0 ? 0 : header.type + 1;
	}

	std::string getValueRepresentation() {
		std::string buffer { };
		auto bytes = valueSize();

		if (bytes > 0) {
			buffer.resize(bytes + 1);
			buffer[0] = (uint8_t)header;
			buffer[1] = value;
			if (bytes > 1) {
				buffer[2] = value >> 8;
			}
			if (bytes > 2) {
				buffer[3] = value >> 16;
			}
			if (bytes > 3) {
				buffer[4] = value >> 24;
			}
		}

		return buffer;
	}

private:
	std::string description;
	std::string unit;
	int32_t value;
};

struct SensorInfo {
	uint16_t identifier : 12;
	uint16_t numberOfTelemetryData : 4;

	SensorInfo &operator = (const uint16_t data) {
		identifier = (data >> 4) & 0xFFF;
		numberOfTelemetryData = data & 0x0F;
		return *this;
	};

	operator uint16_t() const {
		return (identifier << 4) | numberOfTelemetryData;
	}
};

class Sensor {
public:
	SensorInfo sensorInfo;
	std::vector<TelemetryData *> telemetryDataArray;

	void processHeartBeat() { tick++; }

	virtual bool start() = 0;
	virtual std::string getDescription() = 0;
	virtual size_t dataSize() = 0;
	virtual std::string getData() = 0;

private:
	uint8_t tick { 0 }; // 1 tick = 10 ms
	uint16_t refreshRateInMilliSeconds { 50 };
};

#endif /* __SENSOR_H__ */

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

typedef struct {
	uint8_t position : 4;
	uint8_t type : 2;
	uint8_t decimalPointPosition : 2;

	operator uint8_t() const {
		return (position << 4) | (type << 2) | decimalPointPosition;
	}
} TelemetryDataHeader;

enum TelemetryDataType {
	int8_tdt = 0, // [-128, 127]
	int16_tdt, // [-32768, 32767]
	int24_tdt, // [-8388608, 8388607]
	int32_tdt, // [-2147483648, 2147483647]
};

class TelemetryData {
public:
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

	void setValue(int8_t value) {
		this->value = value;
	}

	void setValue(int16_t value) {
		this->value = value;
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
	TelemetryDataHeader header;
	int32_t value;
};

class Sensor {
public:
	void processHeartBeat() { tick++; }

	virtual bool start() = 0;
	virtual std::string getDescription() = 0;
	virtual size_t dataSize() = 0;
	virtual std::string getData() = 0;

private:
	uint8_t tick { 0 }; // 1 tick = 10 ms
	uint16_t refreshRateInMilliSeconds { 50 };
	uint8_t identifier;
};

#endif /* __SENSOR_H__ */

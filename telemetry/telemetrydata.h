/**
 * @file telemetrydata.h
 * @brief Telemetry data type definition.
 *
 * @author Varadi Gyorgy aka bodri
 * Contact: bodri@bodrico.com
 *
 * @bug No known bugs.
 *
 * MIT license, all text above must be included in any redistribution
 *
 */

#ifndef __TELEMETRYDATA_H__
#define __TELEMETRYDATA_H__

#include <string>
#include <vector>

struct TelemetryDataHeader {
	uint8_t position : 3; 				// max. 8 sensor values
	uint8_t type : 3;					// 8 data types
	uint8_t decimalPointPosition : 2;	// max. 3 decimals

	TelemetryDataHeader& operator= (const uint8_t data) {
		position = (data >> 5) & 0x7;
		type = (data >> 2) & 0x7;
		decimalPointPosition = data & 0x3;
		return *this;
	}

	operator uint8_t() const {
		return (position << 5) | (type << 2) | decimalPointPosition;
	}
} __attribute__ ((__packed__));

typedef enum {
	int8_tdt = 0, // [-128, 127]
	int16_tdt, // [-32768, 32767]
	int24_tdt, // [-8388608, 8388607]
	int32_tdt, // [-2147483648, 2147483647]
} TelemetryDataType;

class TelemetryData {
public:
	TelemetryDataHeader header;

	TelemetryData(uint8_t position, std::string description, std::string unit, uint8_t dataType, uint8_t decimalPointPosition) :
		description(description),
		unit(unit)
	{
		TelemetryDataHeader header;
		header.position = position & 0x7;
		header.type = TelemetryDataType(dataType);
		header.decimalPointPosition = decimalPointPosition & 0x3;
		this->header = header;
	}

	void setValue(int32_t value) {
		this->value = value;
	}

	int32_t getValue() {
		return this->value;
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

#endif /* __TELEMETRYDATA_H__ */

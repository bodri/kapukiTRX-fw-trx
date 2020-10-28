/**
 * @file crsf.h
 * @brief Handle Crossfire protocol.
 *
 * @author Varadi, Gyorgy, aka bodri
 * Contact: bodri@bodrico.com
 *
 * @bug No known bugs.
 *
 * MIT license, all text above must be included in any redistribution
 *
 */

#ifndef __CRSF_H__
#define __CRSF_H__

#include "channel.h"
#include "usart.h"
#include "crc.h"

#include "telemetry.h"

#include <stdint.h>

#define CRSF_MAX_CHANNELS   16U      // Maximum number of channels from crossfire data stream
#define CRSF_FRAMELEN_MAX   64U      // Maximum possible frame length

class Crossfire {
public:
	Crossfire(UART_HandleTypeDef *serialPort, CRC_HandleTypeDef *crc);

	void setTelemetry(bool tracking, Telemetry *telemetry) { this->tracking = tracking; this->telemetry = telemetry; }
	void processRxComplete(UART_HandleTypeDef *huart, volatile bool *packetReceived) { if (huart->Instance == serialPort->Instance) *packetReceived = true; }
	void processSerialError(UART_HandleTypeDef *huart);
	void decodePacket(uint8_t *buffer, size_t maxBufferLength, ChannelData &channelData, volatile bool *packetReceived);

private:
    struct Frame {
        uint8_t deviceAddress;
        uint8_t length;
        uint8_t payload[CRSF_FRAMELEN_MAX - 2]; // Contains also type and crc
    } __attribute__ ((__packed__));

    struct LinkStatisticsFrame {
    	const uint8_t type { 0x14 };
        uint8_t rxRssi1;		// ( dBm * -1 )
        uint8_t rxRssi2;		// ( dBm * -1 )
        uint8_t rxQuality;		// Package success rate / Link quality ( % )
        int8_t rxSnr;			// ( db )
        uint8_t antenna;		// Diversity active antenna ( enum ant. 1 = 0, ant. 2 )
        uint8_t rfMode;			// ( enum 4fps = 0 , 50fps, 150hz)
        uint8_t txPower;		// ( enum 0mW = 0, 10mW, 25 mW, 100 mW, 500 mW, 1000 mW, 2000mW )
        uint8_t txRssi;			// ( dBm * -1 )
        uint8_t txQuality;		// Downlink package success rate / Link quality ( % )
        int8_t txSnr;			// ( db )
    } __attribute__ ((__packed__));

    struct GpsFrame {			// curious fact, calling this GPS makes sizeof(GPS) return 1!
    	const uint8_t type { 0x02 };
        int32_t latitude;		// ( degree / 10`000`000 )
        int32_t longitude;		// (degree / 10`000`000 )
        uint16_t groundSpeed;	// ( km/h / 100 )
        uint16_t heading;		// ( degree / 100 )
        uint16_t altitude;		// ( meter - 1000m offset )
        uint8_t satellites;		// in use ( counter )
    } __attribute__ ((__packed__));

    struct BatteryFrame {
    	const uint8_t type { 0x08 };
        uint16_t voltage;		// ( mV * 100 )
        uint16_t current;		// ( mA * 100 )
        uint8_t capacity[3];	// ( mAh )
        uint8_t remaining;		// ( percent )
    } __attribute__ ((__packed__));

    struct AttitudeFrame {
    	const uint8_t type { 0x1E };
        int16_t pitchangle;		// ( rad * 10000 )
        int16_t rollAngle;		// ( rad * 10000 )
        int16_t yawAngle;		// ( rad * 10000 )
    } __attribute__ ((__packed__));

    struct FlightModeFrame {
    	const uint8_t type { 0x21 };
        char flightMode[16];	// ( Null-terminated string )
    } __attribute__ ((__packed__));

    struct VarioFrame {
    	const uint8_t type { 0x07 };
    	int16_t verticalSpeed;	// ( m/s )
    } __attribute__ ((__packed__));

	struct Channels11Bit {
		// 176 bits of data (11 bits per channel * 16 channels) = 22 bytes.
#if __BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__
	#error "Only supported on little-endian architectures"
#endif
		uint16_t ch0 :11;
		uint16_t ch1 :11;
		uint16_t ch2 :11;
		uint16_t ch3 :11;
		uint16_t ch4 :11;
		uint16_t ch5 :11;
		uint16_t ch6 :11;
		uint16_t ch7 :11;
		uint16_t ch8 :11;
		uint16_t ch9 :11;
		uint16_t ch10 :11;
		uint16_t ch11 :11;
		uint16_t ch12 :11;
		uint16_t ch13 :11;
		uint16_t ch14 :11;
		uint16_t ch15 :11;
	} __attribute__ ((__packed__));

	UART_HandleTypeDef *serialPort;
	CRC_HandleTypeDef *crc;

	uint8_t serialBuffer[CRSF_FRAMELEN_MAX] { 0 };
	uint8_t telemetryBuffer[CRSF_FRAMELEN_MAX] { 0 };
	volatile bool frameError { false };

	LinkStatisticsFrame linkStatistics;

	uint8_t telemetryTypeCounter { 0 };
	Telemetry *telemetry;
	bool tracking;

	void decode11BitChannels(const uint8_t *data, uint8_t numberOfChannels, ChannelData &channelData, uint16_t mult, uint16_t div, uint16_t offset);
	void sendBackTelemetry();
	void sendLinkStatistics();
};


#endif /* __CRSF_H__ */

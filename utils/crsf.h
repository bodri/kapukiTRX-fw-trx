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

#include <stdint.h>

#define CRSF_MAX_CHANNELS   16U      // Maximum number of channels from crsf datastream
#define CRSF_FRAMELEN_MAX   64U      // maximum possible framelength


class Crossfire {
public:

	void decodePacket(uint8_t *buffer, size_t maxBufferLength, ChannelData &channelData);

private:
	typedef struct __attribute__ ((__packed__)) {
		// 176 bits of data (11 bits per channel * 16 channels) = 22 bytes.
#if __BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__
	#error "Only supported on little-endian architectures"
#endif
		uint32_t ch0 :11;
		uint32_t ch1 :11;
		uint32_t ch2 :11;
		uint32_t ch3 :11;
		uint32_t ch4 :11;
		uint32_t ch5 :11;
		uint32_t ch6 :11;
		uint32_t ch7 :11;
		uint32_t ch8 :11;
		uint32_t ch9 :11;
		uint32_t ch10 :11;
		uint32_t ch11 :11;
		uint32_t ch12 :11;
		uint32_t ch13 :11;
		uint32_t ch14 :11;
		uint32_t ch15 :11;
	} Channels11Bit;

	uint8_t serialBuffer[26] { 0 };
	volatile bool packetReceived { false };
	volatile bool frameError { false };

	void decode11BitChannels(const uint8_t *data, uint8_t numberOfChannels, ChannelData &channelData, uint16_t mult, uint16_t div, uint16_t offset);
};


#endif /* __CRSF_H__ */

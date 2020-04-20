/**
 * @file channel.cpp
 * @brief Channel data representation.
 *
 * @author Varadi, Gyorgy, aka bodri
 * Contact: bodri@bodrico.com
 *
 * @bug No known bugs.
 *
 * MIT license, all text above must be included in any redistribution
 *
 */

#include "channel.h"

ChannelData::ChannelData (const Packet& packet) {
	for (auto pointer = packet.payload; pointer < packet.payload + sizeof(packet.payload); pointer += 3) {
		Channel channel1 = pointer[0] + (((pointer[2] >> 4) & 0x0F) << 8);
		Channel channel2 = pointer[1] + ((pointer[2] & 0x0F) << 8);
		channels.push_back(channel1);
		channels.push_back(channel2);
	}
}

void ChannelData::fillRawChannelData(Packet &packet) {
	if (sizeof(packet) < channels.size() / 2 * 3) {
		return;
	}

	uint8_t *rawData = packet.payload;
	for (size_t i = 0, j = 0; i < channels.size(); i++, j += 3) {
		uint16_t value = channels[i++].value;
		*(rawData + j) = (uint8_t)value;
		*(rawData + j + 2) = (uint8_t)((value >> 8) & 0x0F);

		value = channels[i].value;
		*(rawData + j + 1) = (uint8_t)value;
		*(rawData + j + 2) &= (uint8_t)((value >> 4) & 0xF0);
	}
}

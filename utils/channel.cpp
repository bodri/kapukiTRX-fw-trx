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

ChannelData::ChannelData(uint8_t numberOfChannels) {
	for (int i = 0; i < numberOfChannels; i++) {
		Channel *channel = new Channel(0);
		channels.push_back(channel);
	}
}

ChannelData::~ChannelData() {
	for (auto channel : channels) {
		delete channel;
	}
}

ChannelData &ChannelData::operator= (const Packet& packet) {
	int channelPointer { 0 };
	for (auto pointer = packet.payload; pointer < packet.payload + sizeof(packet.payload); pointer += 3) {
		channels[channelPointer++]->value = pointer[0] | ((pointer[2] & 0x0F) << 8);
		channels[channelPointer++]->value = pointer[1] | ((pointer[2] & 0xF0) << 4);
	}

	return *this;
}

void ChannelData::fillRawChannelData(Packet &packet) {
	if (sizeof(packet.payload) < channels.size() / 2 * 3) {
		return;
	}

	uint8_t *rawData = packet.payload;
	for (size_t i = 0, j = 0; i < channels.size(); j += 3) {
		uint16_t value = channels[i++]->value;
		*(rawData + j) = (uint8_t)value;
		*(rawData + j + 2) = (uint8_t)((value >> 8) & 0x0F);

		value = channels[i++]->value;
		*(rawData + j + 1) = (uint8_t)value;
		*(rawData + j + 2) |= (uint8_t)((value >> 8) & 0x0F) << 4;
	}
}

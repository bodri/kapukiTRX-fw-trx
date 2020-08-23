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
	channels.reserve(numberOfChannels);
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
	uint8_t sizeOfPayload = packet.size - sizeof(packet.status);
	int channelPointer { 0 };
	for (auto pointer = packet.payload; pointer < packet.payload + sizeOfPayload; pointer += 3) {
		channels[channelPointer++]->value = pointer[1] | ((pointer[2] & 0xF0) << 4);
		channels[channelPointer++]->value = pointer[0] | ((pointer[2] & 0x0F) << 8);
	}

	return *this;
}

void ChannelData::fillRawChannelData(Packet &packet) {
	if (sizeof(packet.payload) < channels.size() / 2 * 3) {
		return;
	}

	uint8_t *rawData = packet.payload;
	for (size_t i = 0; i < channels.size(); rawData	+= 3) {
		uint16_t value = channels[i++]->value;
		*(rawData) = (uint8_t)value;
		*(rawData + 2) = (uint8_t)((value >> 8) & 0x0F);

		value = channels[i++]->value;
		*(rawData + 1) = (uint8_t)value;
		*(rawData + 2) |= (uint8_t)((value >> 4) & 0xF0);
	}
}

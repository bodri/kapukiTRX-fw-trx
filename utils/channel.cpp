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
	switch (packet.status.packetType) {
	case NORMAL:
		loadNormalChannelData(packet.payload);
		break;
	case OPENTX:
		loadOpenTxChannelData(packet.payload);
		break;
	default:
		break;
	}

	return *this;
}

void ChannelData::fillRawChannelData(Packet &packet) {
	if (sizeof(packet.payload) < channels.size() / 2 * 3) {
		return;
	}

	switch (packet.status.packetType) {
	case NORMAL:
		fillNormalChannelData(packet.payload);
		break;
	case OPENTX:
		fillOpenTxChannelData(packet.payload);
		break;
	default:
		break;
	}
}

void ChannelData::loadNormalChannelData(const uint8_t *payload) {
	const Packet::NormalChannels *receivedChannelData = reinterpret_cast<const Packet::NormalChannels *>(payload);
	*channels[0] = receivedChannelData->ch0;
	*channels[1] = receivedChannelData->ch1;
	*channels[2] = receivedChannelData->ch2;
	*channels[3] = receivedChannelData->ch3;
	*channels[4] = receivedChannelData->ch4;
	*channels[5] = receivedChannelData->ch5;
	*channels[6] = receivedChannelData->ch6;
	*channels[7] = receivedChannelData->ch7;
	*channels[8] = receivedChannelData->ch8;
	*channels[9] = receivedChannelData->ch9;
	*channels[10] = receivedChannelData->ch10;
	*channels[11] = receivedChannelData->ch11;
	*channels[12] = receivedChannelData->ch12;
	*channels[13] = receivedChannelData->ch13;
	*channels[14] = receivedChannelData->ch14;
	*channels[15] = receivedChannelData->ch15;
}

void ChannelData::loadOpenTxChannelData(const uint8_t *payload) {
	const Packet::OpenTxChannels *receivedChannelData = reinterpret_cast<const Packet::OpenTxChannels *>(payload);
	*channels[0] = receivedChannelData->ch0;
	*channels[1] = receivedChannelData->ch1;
	*channels[2] = receivedChannelData->ch2;
	*channels[3] = receivedChannelData->ch3;
	*channels[4] = receivedChannelData->ch4;
	*channels[5] = receivedChannelData->ch5;
	*channels[6] = receivedChannelData->ch6;
	*channels[7] = receivedChannelData->ch7;
	*channels[8] = receivedChannelData->ch8;
	*channels[9] = receivedChannelData->ch9;
	*channels[10] = receivedChannelData->ch10;
	*channels[11] = receivedChannelData->ch11;
	*channels[12] = receivedChannelData->ch12;
	*channels[13] = receivedChannelData->ch13;
	*channels[14] = receivedChannelData->ch14;
	*channels[15] = receivedChannelData->ch15;
}

void ChannelData::fillNormalChannelData(uint8_t *payload) {
	Packet::NormalChannels *payloadChannelData = reinterpret_cast<Packet::NormalChannels *>(payload);
	payloadChannelData->ch0 = *channels[0];
	payloadChannelData->ch1 = *channels[1];
	payloadChannelData->ch2 = *channels[2];
	payloadChannelData->ch3 = *channels[3];
	payloadChannelData->ch4 = *channels[4];
	payloadChannelData->ch5 = *channels[5];
	payloadChannelData->ch6 = *channels[6];
	payloadChannelData->ch7 = *channels[7];
	payloadChannelData->ch8 = *channels[8];
	payloadChannelData->ch9 = *channels[9];
	payloadChannelData->ch10 = *channels[10];
	payloadChannelData->ch11 = *channels[11];
	payloadChannelData->ch12 = *channels[12];
	payloadChannelData->ch13 = *channels[13];
	payloadChannelData->ch14 = *channels[14];
	payloadChannelData->ch15 = *channels[15];
}

void ChannelData::fillOpenTxChannelData(uint8_t *payload) {
	Packet::OpenTxChannels *payloadChannelData = reinterpret_cast<Packet::OpenTxChannels *>(payload);
	payloadChannelData->ch0 = *channels[0];
	payloadChannelData->ch1 = *channels[1];
	payloadChannelData->ch2 = *channels[2];
	payloadChannelData->ch3 = *channels[3];
	payloadChannelData->ch4 = *channels[4];
	payloadChannelData->ch5 = *channels[5];
	payloadChannelData->ch6 = *channels[6];
	payloadChannelData->ch7 = *channels[7];
	payloadChannelData->ch8 = *channels[8];
	payloadChannelData->ch9 = *channels[9];
	payloadChannelData->ch10 = *channels[10];
	payloadChannelData->ch11 = *channels[11];
	payloadChannelData->ch12 = *channels[12];
	payloadChannelData->ch13 = *channels[13];
	payloadChannelData->ch14 = *channels[14];
	payloadChannelData->ch15 = *channels[15];
}

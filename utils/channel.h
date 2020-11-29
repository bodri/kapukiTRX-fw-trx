/**
 * @file channel.h
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

#ifndef __CHANNEL_H__
#define __CHANNEL_H__

#include "rflink.h"

#include <stdint.h>
#include <vector>

class Channel {
public:
	Channel(uint16_t value) : value(value) { }
	~Channel() { }

	Channel &operator = (const uint16_t data) {
		value = data;
		return *this;
	};

	operator uint16_t() const {
		return value;
	}

private:
	uint16_t value;
};

class ChannelData {
public:
	ChannelData(uint8_t numberOfChannels);
	~ChannelData();

	ChannelData &operator= (const Packet& packet);

	Channel *operator[](int index) { return channels.at(index); }

	void fillRawChannelData(Packet &packet);

	std::vector<Channel *> channels;

private:
	void loadNormalChannelData(const uint8_t *payload);
	void loadOpenTxChannelData(const uint8_t *payload);
	void fillOpenTxChannelData(uint8_t *payload);
	void fillNormalChannelData(uint8_t *payload);
};

#endif // __CHANNEL_H__

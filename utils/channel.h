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
	Channel(uint16_t value):value(value) { }
	~Channel() { }

	uint16_t value;
};

class ChannelData {
public:
	ChannelData() { }
	~ChannelData() { }

	ChannelData (const Packet& packet);
	ChannelData& operator= (const Packet& packet) {return *this;}

	Channel& operator[](int index) { return channels.at(index);}

	void fillRawChannelData(Packet &packet);

	std::vector<Channel> channels;
};

#endif // __CHANNEL_H__

/*
 * channel.h
 *
 *  Created on: 17 Apr 2020
 *      Author: gvaradi
 */

#ifndef CHANNEL_H
#define CHANNEL_H

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
	ChannelData() { };
	~ChannelData() { };

	ChannelData (const Packet& packet);
	ChannelData& operator= (const Packet& packet) {return *this;}

	Channel& operator[](int index) { return channels.at(index);}

	void fillRawChannelData(Packet &packet);

	std::vector<Channel> channels;
};

#endif /* CHANNEL_H */

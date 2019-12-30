/*
 * patterngen.h
 *
 *  Created on: Nov 10, 2018
 *      Author: bodri
 */

#ifndef PATTERNGEN_H_
#define PATTERNGEN_H_

#include <stdint.h>
#include <vector>
#include <array>

class PatternGenerator {
public:
	PatternGenerator(const uint8_t *hopSet, size_t size);
	uint8_t nextHop(void);
	uint8_t currentChannel(void);

	static const size_t maximumNumberOfChannels { 79 };

private:
	std::vector<uint8_t> hopSet { };
	size_t usedNumberOfHops { 1 };
	size_t hopIndex { 0 };

	static const std::array<uint8_t, maximumNumberOfChannels> defaultHopSet;
};



#endif /* PATTERNGEN_H_ */

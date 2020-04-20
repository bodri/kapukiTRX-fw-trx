/**
 * @file patterngen.h
 * @brief Hopping sequence/pattern generator.
 *
 * @author Varadi, Gyorgy, aka bodri
 * Contact: bodri@bodrico.com
 *
 * @bug No known bugs.
 *
 * MIT license, all text above must be included in any redistribution
 *
 */

#ifndef __PATTERNGEN_H__
#define __PATTERNGEN_H__

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



#endif // __PATTERNGEN_H__

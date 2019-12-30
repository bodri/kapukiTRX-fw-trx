/*
 * patterngen.cpp
 *
 *  Created on: Nov 10, 2018
 *      Author: bodri
 */


#include "patterngen.h"
#include "main.h"

#include <string.h>

PatternGenerator::PatternGenerator(const uint8_t *hopSet, size_t size) {
    usedNumberOfHops = std::max((size_t)1, std::min(sizeof(PatternGenerator::defaultHopSet), size));

    for (size_t i = 0; i < usedNumberOfHops; i++) {
    	this->hopSet.push_back(hopSet && size > 0 ? hopSet[i] : PatternGenerator::defaultHopSet[i]);
    }

    this->hopSet.shrink_to_fit();
}

uint8_t PatternGenerator::currentChannel(void) {
	return hopSet[hopIndex];
}

//
// Private functions
//
uint8_t PatternGenerator::nextHop() {
	if (hopIndex > hopSet.size()) { return 0; }

    uint8_t nextHop = hopSet[hopIndex++];
    if (hopIndex >= usedNumberOfHops) {
        hopIndex = 0;
    }

    return nextHop;
}

const std::array<uint8_t, PatternGenerator::maximumNumberOfChannels> PatternGenerator::defaultHopSet { 37, 18, 76, 16, 51, 38, 49, 72, 59, 79, 60, 46, 21, 63, 34, 1, 73, 0, 13, 3, 58, 54, 69, 52, 33, 15, 75, 22, 9, 71, 78, 7, 64, 31, 42, 70, 68, 10, 44, 50, 30, 53, 57, 35, 55, 2, 41, 62, 11, 24, 48, 25, 20, 65, 23, 8, 40, 74, 56, 28, 36, 32, 14, 4, 29, 17, 67, 19, 61, 39, 27, 77, 5, 26, 47}; //, 6, 45, 12, 43, 66 };



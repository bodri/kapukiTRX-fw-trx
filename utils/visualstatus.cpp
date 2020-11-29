/**
 * @file visualstatus.cpp
 * @brief Shows the RF link status using an RGB LED.
 *
 * @author Varadi, Gyorgy, aka bodri
 * Contact: bodri@bodrico.com
 *
 * @bug No known bugs.
 *
 * MIT license, all text above must be included in any redistribution
 *
 */

#include <visualstatus.h>

void VisualStatus::processHeartBeat(TIM_HandleTypeDef *htim) {
	if (htim != heartBeatTimer) { return; }

	tick++;
}

void VisualStatus::runLoop(void) {
	if (tick > 100) {
		tick = 0;
	}

	if (status != previousStatus) {
		previousStatus = status;
		redLed.high();
		greenLed.high();
		blueLed.high();
	}

	switch (status) {
	case WAITING_FOR_CONNECTION:
		tick < 50 ? greenLed.low() : greenLed.high();
		break;
	case TRACKING:
		greenLed.low();
		break;
	case CONNECTION_LOST:
		tick < 50 ? redLed.low() : redLed.high();
		break;
	case ERROR_GENERAL:
		redLed.low();
		break;
	default:
		break;
	}
}

void VisualStatus::setStatus(LedStatus status) {
	if (status == previousStatus) { return; }
	this->status = status;
	this->tick = 0;
}

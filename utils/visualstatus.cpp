/*
 * visualstatus.cpp
 *
 *  Created on: Dec 30, 2019
 *      Author: bodri
 */

#include <visualstatus.h>

void VisualStatus::processHeartBeat(TIM_HandleTypeDef *htim) {
	if (htim != heartBeatTimer) { return; }

	tick++;
}

void VisualStatus::runLoop(void) {
	if (tick > 50) {
		tick = 0;
	}

	if (status != previousStatus) {
		previousStatus = status;
		redLed.high();
		greenLed.high();
//		blueLed.high();
	}

	switch (status) {
	case WAITING_FOR_CONNECTION:
		tick < 25 ? greenLed.low() : greenLed.high();
		break;
	case TRACKING:
		greenLed.low();
		break;
	case CONNECTION_LOST:
		tick < 25 ? redLed.low() : redLed.high();
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

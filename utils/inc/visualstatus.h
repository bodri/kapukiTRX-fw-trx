/*
 * visualstatus.h
 *
 *  Created on: Dec 30, 2019
 *      Author: bodri
 */

#ifndef VISUALSTATUS_H_
#define VISUALSTATUS_H_

#include "pin.hpp"

typedef enum {
	WAITING_FOR_CONNECTION,
	TRACKING,
	CONNECTION_LOST,
	ERROR_GENERAL
} LedStatus;

class VisualStatus {
public:
	VisualStatus(TIM_HandleTypeDef *heartBeatTimer, Pin redLed, Pin greenLed, Pin blueLed) :
			heartBeatTimer(heartBeatTimer),
			redLed(redLed),
			greenLed(greenLed),
			blueLed(blueLed)
	{ } ;

	~VisualStatus() { };

	void processHeartBeat(TIM_HandleTypeDef *htim);
	void runLoop(void);

	void setStatus(LedStatus status);

private:
	TIM_HandleTypeDef *heartBeatTimer;
	Pin redLed;
	Pin greenLed;
	Pin blueLed;

	volatile int tick { 0 };
	LedStatus status { WAITING_FOR_CONNECTION };
	LedStatus previousStatus { WAITING_FOR_CONNECTION };
};

#endif // VISUALSTATUS_H_

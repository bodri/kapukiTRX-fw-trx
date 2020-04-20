/**
 * @file visualstatus.h
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

#ifndef __VISUALSTATUS_H__
#define __VISUALSTATUS_H__

#include <pin.hpp>

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

#endif // __VISUALSTATUS_H__

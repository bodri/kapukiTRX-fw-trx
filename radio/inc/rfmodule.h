/*
 * rfmodule.h
 *
 *  Created on: Nov 10, 2018
 *      Author: bodri
 */

#ifndef RFMODULE_H_
#define RFMODULE_H_

#include <functional>
#include <vector>

class RfModule {
public:
	RfModule() { };
	virtual ~RfModule() { };

	virtual void init(void) = 0;
	virtual void standBy(void) = 0;
	virtual void setAddress(uint16_t) = 0;
	virtual void setChannel(uint8_t) = 0;
	virtual void enterRx(void) = 0;
	virtual void send(uint8_t *, uint8_t) = 0;

	std::function<void(void)> onSyncWordDone;
	std::function<void(void)> onRxDone;
	std::function<void(void)> onTxDone;
};

#endif /* RFMODULE_H_ */

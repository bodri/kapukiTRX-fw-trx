/**
 * @file rfmodule.h
 * @brief An RF module protocol.
 *
 * @author Varadi, Gyorgy, aka bodri
 * Contact: bodri@bodrico.com
 *
 * @bug No known bugs.
 *
 * MIT license, all text above must be included in any redistribution
 *
 */

#ifndef __RFMODULE_H__
#define __RFMODULE_H__

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

	std::function<void(void)> onRxDone;
	std::function<void(void)> onTxDone;
	std::function<void(void)> onTimeout;
};

#endif // __RFMODULE_H__

/*
 * rflink.h
 *
 *  Created on: Nov 9, 2018
 *      Author: bodri
 */

#ifndef RFLINK_H_
#define RFLINK_H_

#include "main.h"
#include "sx1280.h"
#include "patterngen.h"

#include <stdint.h>
#include <map>

typedef struct {
	struct {
		uint16_t packetType : 3;
		uint16_t rfu : 3;
		uint16_t packetNumber : 10;
	} status;

	uint8_t payload[39];
} Packet;

typedef struct {
	uint8_t ch1Lsb;
	uint8_t ch2Lsb;
	struct {
		uint8_t ch1Msb : 4;
		uint8_t ch2Msb : 4;
	} ch12Msb;

	uint8_t ch3Lsb;
	uint8_t ch4Lsb;
	struct {
		uint8_t ch3Msb : 4;
		uint8_t ch4Msb : 4;
	} ch34Msb;

	uint8_t *begin() { return &ch1Lsb; }
	uint8_t *end() { return &ch4Lsb + 2; }
} ServoData;

typedef enum {
				INIT,
				SEND_OR_ENTER_RX,
	// Tx							// Rx
	TRANSMITTED,					WAITING_FOR_SYNC,
									SYNC_RECEIVED,
									WAITING_FOR_RECEIVE,
									RECEIVED,
				DONE,
				WAITING_FOR_NEXT_HOP
} LinkState;

typedef enum {
	NO_IRQ,
	MODULE1_IRQ,
	MODULE2_IRQ
} IrqSource;

class RfLink {
public:
	RfLink(TIM_HandleTypeDef *heartBeatTimer, bool transmitter) :
		heartBeatTimer(heartBeatTimer),
		transmitter(transmitter)
	{ };

	~RfLink() {
		delete patternGenerator;
		delete rf1Module;
		delete rf2Module;
	};

	void init();
	void processHeartBeat(TIM_HandleTypeDef *htim);
	void processIrqs(uint16_t pin);
	void runLoop(void);

	std::function<void(Packet &packet)> onTransmit;
	std::function<void(Packet &packet)> onTransmitTelemetry;
	std::function<void(Packet &packet)> onReceive;
	std::function<void(Packet &packet)> onReceiveTelemetry;
	std::function<void(bool tracking)> onLinkStatusChange;

	SX1280 *rf1Module;
	SX1280 *rf2Module;

private:
	TIM_HandleTypeDef *heartBeatTimer;

	bool transmitter;
	bool tracking { false };

	PatternGenerator *patternGenerator;

	Pin rf1TxEnable { Pin(RF1TXEN_GPIO_Port, RF1TXEN_Pin) };
	Pin rf1RxEnable { Pin(RF1RXEN_GPIO_Port, RF1RXEN_Pin) };

	Pin rf2TxEnable { Pin(RF2TXEN_GPIO_Port, RF2TXEN_Pin) };
	Pin rf2RxEnable { Pin(RF2RXEN_GPIO_Port, RF2RXEN_Pin) };

	volatile LinkState state { INIT };
	uint16_t packetNumber;

	volatile bool heartBeatTimeout { false };
	volatile IrqSource lastIrqSource { NO_IRQ };
	bool useRf1 { true };
	uint32_t lostSync { 0 };
	uint32_t lostPacket { 0 };
	std::map<uint8_t, int> failuresPerChannel { };
	uint16_t timerWhenSyncReceived { 746 }; // Measured

	const uint8_t lostSyncTreshold { 20 };
	const uint8_t downLinkFrequency { 4 };

	bool upLink(void);
	void setTracking(bool tracking);
	void adjustTimerToTrackTx(void);
	void registerLostPacket(void);
	void sendPacket(void);
	void enterRx(void);
};

#endif /* RFLINK_H_ */

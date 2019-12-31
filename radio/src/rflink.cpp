/*
 * rflink.cpp
 *
 *  Created on: Nov 9, 2018
 *      Author: bodri
 */

#include "rflink.h"
#include "spi.h"

#include <array>
#include <vector>

void RfLink::init() {
	this->patternGenerator = new PatternGenerator(NULL, PatternGenerator::maximumNumberOfChannels);

	// init RF mobule #1
	Pin rf1NresetPin(RF1NRESET_GPIO_Port, RF1NRESET_Pin);
	Pin rf1NcsPin(RF1NSS_GPIO_Port, RF1NSS_Pin);
	Pin rf1BusyPin(RF1BUSY_GPIO_Port, RF1BUSY_Pin);
	this->rf1Module = new SX1280(&hspi2, rf1NresetPin, rf1NcsPin, rf1BusyPin);

	rf1Module->onTxDone = [this]() {
		rf1TxEnable.low();
		state = TRANSMITTED;
	};

	rf1Module->onSyncWordDone = [this]() {
	    state = SYNC_RECEIVED;
	};

	rf1Module->onRxDone = [this]() {
		rf1RxEnable.low();
		state = RECEIVED;
	};

	rf1Module->init();
	rf1Module->setAddress(0x6969);

	// init RF mobule #2
	Pin rf2NresetPin(RF2NRESET_GPIO_Port, RF2NRESET_Pin);
	Pin rf2NcsPin(RF2NSS_GPIO_Port, RF2NSS_Pin);
	Pin rf2BusyPin(RF2BUSY_GPIO_Port, RF2BUSY_Pin);
	this->rf2Module = new SX1280(&hspi1, rf2NresetPin, rf2NcsPin, rf2BusyPin);

	rf2Module->onTxDone = [this]() {
		rf2TxEnable.low();
		state = TRANSMITTED;
	};

	rf2Module->onSyncWordDone = [this]() {
	    state = SYNC_RECEIVED;
	};

	rf2Module->onRxDone = [this]() {
		rf2RxEnable.low();
		state = RECEIVED;
	};

	rf2Module->init();
	rf2Module->setAddress(0x6969);

    if (transmitter) {
    	uint16_t txIrqMask { IRQ_TX_DONE };
    	rf1Module->setDioIrqParams(txIrqMask, txIrqMask, IRQ_RADIO_NONE, IRQ_RADIO_NONE);
    	rf2Module->setDioIrqParams(txIrqMask, txIrqMask, IRQ_RADIO_NONE, IRQ_RADIO_NONE);
    } else {
    	uint16_t rxIrqMask { IRQ_RX_DONE | IRQ_SYNCWORD_VALID };
    	rf1Module->setDioIrqParams(rxIrqMask, rxIrqMask, IRQ_RADIO_NONE, IRQ_RADIO_NONE);
    	rf2Module->setDioIrqParams(rxIrqMask, rxIrqMask, IRQ_RADIO_NONE, IRQ_RADIO_NONE);
	}
}

void RfLink::processHeartBeat(TIM_HandleTypeDef *htim) {
	if (htim != heartBeatTimer) { return; }

	heartBeatTimeout = true;
}

void RfLink::processIrqs(uint16_t pin) {
//	static const Pin rf1irqPin(RF1IRQ_GPIO_Port, RF1IRQ_Pin);
//	static const Pin rf2irqPin(RF2IRQ_GPIO_Port, RF2IRQ_Pin);
	if (pin == RF1IRQ_Pin) {
		lastIrqSource = MODULE1_IRQ;
	} else if (pin == RF2IRQ_Pin) {
		lastIrqSource = MODULE2_IRQ;
	}
}

void RfLink::runLoop(void) {

	if (lastIrqSource != NO_IRQ) {
		// 56 us
		if (lastIrqSource == MODULE1_IRQ) {
			rf1Module->processIrqs();
		} else if (lastIrqSource == MODULE2_IRQ) {
			rf2Module->processIrqs();
		} else {

		}

		lastIrqSource = NO_IRQ;
	}

	if (heartBeatTimeout) {
		// tracking: 28 us, acquisition: ~140us
//		LL_IWDG_ReloadCounter(IWDG);
		heartBeatTimeout = false;
//		HAL_SPI_Abort(&hspi1);
//		HAL_GPIO_WritePin(RF1CS_GPIO_Port, RF1CS_Pin, GPIO_PIN_SET);

		packetNumber++;
		if (packetNumber >= 1000) {
			packetNumber = 0;
		}

		if (!transmitter && !tracking && packetNumber % 3 != 0) {
			return;
		}

		rf1RxEnable.low();
		rf1TxEnable.low();
		rf2RxEnable.low();
		rf2TxEnable.low();

		if (state != WAITING_FOR_NEXT_HOP) {
			// 70 us
			registerLostPacket();
		}

		useRf1 = !useRf1;
		state = SEND_OR_ENTER_RX;
	}

	switch (state) {
	case SEND_OR_ENTER_RX: {
		// 49 us
		uint8_t nextChannel = patternGenerator->nextHop();
		rf1Module->setChannel(nextChannel);
		rf2Module->setChannel(nextChannel);

		// sendPacket xxx us, enterRx: 153 us
		upLink() ? sendPacket() : enterRx();
		state = WAITING_FOR_SYNC;
	}
		break;
	case TRANSMITTED:
		state = DONE;
		break;
	case SYNC_RECEIVED:
		lostSync = 0;
	    adjustTimerToTrackTx();
	    state = WAITING_FOR_RECEIVE;
	    break;
	case RECEIVED: {
		// xxx us
		lostPacket = 0;

		uint8_t payload[127];
		uint8_t maxSize = sizeof(payload);
		uint8_t size;
		memset(payload, 0, 127);
		rf1Module->getPayload(payload, &size, maxSize);
		if (size == 41) {
			if (onReceive != nullptr) {
				Packet packet;
				memcpy(&packet.status, &payload[0], 2);
				memcpy(&packet.payload[0], &payload[2], 6);
				onReceive(packet);
			}
		} else {
			rf2Module->getPayload(payload, &size, maxSize);
			if (size == 41) {
				if (onReceive != nullptr) {
					Packet packet;
					memcpy(&packet.status, &payload[0], 2);
					memcpy(&packet.payload[0], &payload[2], 6);
					onReceive(packet);
				}
			}
		}
		state = DONE;
	}
		break;
	case DONE:
//		if (packetNumber == 255) {
//			float per = (float)failuresPerChannel[patternGenerator->currentChannel()];
//			for (int i = 0; i < 10; i++) { }
//		}

//		setTracking(tracking);
//		if (lostPacket > 0) {
//			printf("%lu lost packet(s)!\n", lostPacket);
//			lostPacket = 0;
//		}

//		Packet *packet = rf1Buffer.current;
//
//		if (packet->payload.packetNumber == 100) {
//			printf("Packet: %.20s\n", packet->payload.buffer);
//		}
//
//		printf("Packet %d, RSSI: %ddBm, lqi: %d%%\n",
//				packet->payload.packetNumber, rssiPowerLevel(packet->status.rssi),
//				lqiPercentage(packet->status.linkQuality.lqi));
		state = WAITING_FOR_NEXT_HOP;
		break;
	default:
		break;
	}
}

//
// Private methods
//

bool RfLink::upLink(void) {
	if (tracking) {
		// Every 4th hop is a down-link
//		return ((transmitter && packetNumber % 4) || (!transmitter && !(packetNumber % 4)));
		return transmitter;
	} else {
		// no down-link during acquisition
		return transmitter;
	}
}

void RfLink::setTracking(bool tracking) {
	if (this->tracking != tracking) {
		this->tracking = tracking;

		if (onLinkStatusChange != nullptr) {
			onLinkStatusChange(tracking);
		}
	}
}

void RfLink::adjustTimerToTrackTx(void) {
	__HAL_TIM_SET_COUNTER(heartBeatTimer, timerWhenSyncReceived);
	setTracking(true);
}

void RfLink::registerLostPacket(void) {
	lostSync++;
	lostPacket++;

	if (lostSync > (transmitter ? (lostSyncTreshold / downLinkFrequency) : lostSyncTreshold)) {
		setTracking(false);
		auto currentChannel = patternGenerator->currentChannel();
		auto item = failuresPerChannel.find(currentChannel);
		if (item != failuresPerChannel.end()) {
			failuresPerChannel[currentChannel]++;
		} else {
			failuresPerChannel[currentChannel] = 1;
		}
	}
}

void RfLink::sendPacket(void) {
	if (onTransmit == nullptr) { return; }

	SX1280 *rfModule = useRf1 ? rf1Module : rf2Module;
	useRf1 ? rf1TxEnable.high() : rf2TxEnable.high();
    Packet packet { 0 };
    packet.status.packetNumber = packetNumber;
    onTransmit(packet);
    rfModule->send((uint8_t *)&packet, sizeof(Packet));
}

void RfLink::enterRx(void) {
	rf1RxEnable.high();
	rf2RxEnable.high();
    rf1Module->enterRx();
    rf2Module->enterRx();
}

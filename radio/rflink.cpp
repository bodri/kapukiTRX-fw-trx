/**
 * @file rflink.cpp
 * @brief Handles the RF link.
 *
 * @author Varadi, Gyorgy, aka bodri
 * Contact: bodri@bodrico.com
 *
 * @bug No known bugs.
 *
 * MIT license, all text above must be included in any redistribution
 *
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
		if (state == IDLE) {
			state = TRANSMITTED;
		}
	};

	rf1Module->onRxDone = [this]() {
		rf1RxEnable.low();
		if (state == IDLE) {
			state = RECEIVED;
		}
	};

	rf1Module->onTimeout = [this]() {
		rf1RxEnable.low();
		rf1TxEnable.low();
		state = TIMEOUT;
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
		if (state == IDLE) {
			state = TRANSMITTED;
		}
	};

	rf2Module->onRxDone = [this]() {
		rf2RxEnable.low();
		if (state == IDLE) {
			state = RECEIVED;
		}
	};

	rf2Module->onTimeout = [this]() {
		rf2RxEnable.low();
		rf2TxEnable.low();
		state = TIMEOUT;
	};

	rf2Module->init();
	rf2Module->setAddress(0x6969);

    if (transmitter) {
    	uint16_t txIrqMask { IRQ_RX_DONE | IRQ_TX_DONE };
    	rf1Module->setDioIrqParams(txIrqMask, txIrqMask, IRQ_RADIO_NONE, IRQ_RADIO_NONE);
    	rf2Module->setDioIrqParams(txIrqMask, txIrqMask, IRQ_RADIO_NONE, IRQ_RADIO_NONE);
    } else {
    	uint16_t rxIrqMask { IRQ_RX_DONE | IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT };
    	rf1Module->setDioIrqParams(rxIrqMask, rxIrqMask, IRQ_RADIO_NONE, IRQ_RADIO_NONE);
    	rf2Module->setDioIrqParams(rxIrqMask, rxIrqMask, IRQ_RADIO_NONE, IRQ_RADIO_NONE);
	}
}

void RfLink::processHeartBeat(TIM_HandleTypeDef *htim) {
	if (htim != heartBeatTimer) { return; }

	heartBeatTimeout = true;
	HAL_GPIO_WritePin(SYNC_GPIO_Port, SYNC_Pin, GPIO_PIN_SET);
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
		LL_IWDG_ReloadCounter(IWDG);
		heartBeatTimeout = false;
//		HAL_SPI_Abort(&hspi1);

		packetNumber++;
		if (packetNumber >= 1000) {
			packetNumber = 0;
		}

		if (state != WAITING_FOR_NEXT_HOP) {
			// 70 us
			registerLostPacket();
		}

		if (!transmitter && !tracking && packetNumber % 3 != 0) { // In acquisition mode the receiver slows down by 3
			return;
		}

		rf1RxEnable.low();
		rf1TxEnable.low();
		rf2RxEnable.low();
		rf2TxEnable.low();

		state = START;
	}

	switch (state) {
	case START: {
//		HAL_GPIO_WritePin(SYNC_GPIO_Port, SYNC_Pin, GPIO_PIN_SET);

		// 49 us
		uint8_t nextChannel = patternGenerator->nextHop();
		rf1Module->setChannel(nextChannel);
		rf2Module->setChannel(nextChannel);

		// sendPacket xxx us, enterRx: 153 us
		state = SET_PACKET_PARAMS;
		HAL_GPIO_WritePin(SYNC_GPIO_Port, SYNC_Pin, GPIO_PIN_RESET);

	}
		break;
	case SET_PACKET_PARAMS: {
		bool shouldSend = shouldSendPacket();
		bool telemetryPacket = (transmitter && !shouldSend) || (!transmitter && shouldSend);
		// TODO: optimize: if no change in packet type, we should not set the same again
		setPacketParams(telemetryPacket);
		state = shouldSend ? WAITING_FOR_TX_OFFSET : ENTER_RX;
	}
		break;
	case WAITING_FOR_TX_OFFSET:
		if (__HAL_TIM_GET_COUNTER(heartBeatTimer) > txOffsetInMicroSecond) {
			sendPacket();
			state = IDLE;
		}
		break;
	case ENTER_RX:
		enterRx();
		state = IDLE;
		break;
	case TRANSMITTED: {
//		HAL_GPIO_WritePin(SYNC_GPIO_Port, SYNC_Pin, GPIO_PIN_RESET);
//		uint32_t cucc = __HAL_TIM_GET_COUNTER(heartBeatTimer);
		HAL_GPIO_WritePin(TXORRX_GPIO_Port, TXORRX_Pin, GPIO_PIN_RESET);
		state = DONE;
	}
		break;
	case RECEIVED: {
//		HAL_GPIO_WritePin(SYNC_GPIO_Port, SYNC_Pin, GPIO_PIN_RESET);
		// xxx us
		bool module1Received = validPacket(rf1Module);
		bool module2Received = validPacket(rf2Module);
		if (module1Received || module2Received) {
			lostPacket = 0;
			adjustTimerToTrackTx();
		}

		if (module1Received && loadReceivedPacket(rf1Module)) {
			if (transmitter) {
				HAL_GPIO_WritePin(LEDBLUE_GPIO_Port, LEDBLUE_Pin, GPIO_PIN_RESET);
			}
		} else if (module2Received && loadReceivedPacket(rf2Module)) {
			if (transmitter) {
				HAL_GPIO_WritePin(LEDBLUE_GPIO_Port, LEDBLUE_Pin, GPIO_PIN_SET);
			}
		}
		state = DONE;
	}
		break;
	case TIMEOUT:
		for (int i = 0; i < 100; i++) { }
		state = WAITING_FOR_NEXT_HOP;
		break;
	case DONE: {
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

		HAL_GPIO_WritePin(SYNC_GPIO_Port, SYNC_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(TXORRX_GPIO_Port, TXORRX_Pin, GPIO_PIN_RESET);
		rf1Module->standBy();
		rf2Module->standBy();
		state = WAITING_FOR_NEXT_HOP;

		// Can sleep here
	}
		break;
	default:
		break;
	}
}

//
// Private methods
//
bool RfLink::validPacket(SX1280 *rfModule) {
	PacketStatus_t packetStatus;

	rfModule->getPacketStatus(&packetStatus);
	return (packetStatus.Flrc.ErrorStatus.PacketReceived &&
			!(packetStatus.Flrc.ErrorStatus.AbortError || packetStatus.Flrc.ErrorStatus.CrcError || packetStatus.Flrc.ErrorStatus.LengthError || packetStatus.Flrc.ErrorStatus.SyncError));
}

bool RfLink::loadReceivedPacket(SX1280 *rfModule) {
	uint8_t payload[127];
	uint8_t size;

	rfModule->getPayload(payload, &size, sizeof(payload));
	if (size == sizeof(Packet)) {
		Packet packet;
		memcpy(&packet.status, &payload[0], 2);
		memcpy(&packet.payload[0], &payload[2], sizeof(packet.payload));
		if (transmitter) {
			if (onReceiveTelemetry != nullptr) {
				onReceiveTelemetry(packet);
				return true;
			}
		} else {
			if (onReceive != nullptr) {
				onReceive(packet);

				if (!transmitter) {
					this->packetNumber = packet.status.packetNumber; // sync packetNumber with TX
	//				this->rssiAverage += packetStatus.Flrc.RssiAvg; // save RSSI for telemetry purposes
	//				this->rssiReceivedCount++;
				}

				return true;
			}
		}
	}

	return false;
}

bool RfLink::shouldSendPacket(void) {
	bool telemetryPacket = packetNumber % downLinkFrequency == 0;

	if (transmitter) {
		// Transmitter: Every 4th hop is a down-link
		return !telemetryPacket;
	} else if (tracking) {
		// Receiver: every 4th hop is a down-link in tracking
		return telemetryPacket;
	} else {
		// Receiver: no down-link during acquisition
		return false;
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
	if (!transmitter) {
		__HAL_TIM_SET_COUNTER(heartBeatTimer, timerWhenSyncReceived);
	}
	setTracking(true);
}

void RfLink::registerLostPacket(void) {
	lostPacket++;

	auto currentChannel = patternGenerator->currentChannel();
	auto item = failuresPerChannel.find(currentChannel);
	if (item != failuresPerChannel.end()) {
		failuresPerChannel[currentChannel]++;
	} else {
		failuresPerChannel[currentChannel] = 1;
	}

	if (lostPacket > (transmitter ? (lostPacketTreshold / downLinkFrequency) : lostPacketTreshold)) {
		setTracking(false);
	}
}

void RfLink::sendPacket(void) {
//	HAL_GPIO_WritePin(SYNC_GPIO_Port, SYNC_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(TXORRX_GPIO_Port, TXORRX_Pin, GPIO_PIN_SET);

	useRf1 = !useRf1;
	SX1280 *rfModule = useRf1 ? rf1Module : rf2Module;
	if (useRf1) {
		rf2Module->standBy();
		rf1TxEnable.high();
	} else {
		rf1Module->standBy();
		rf2TxEnable.high();
	}

    Packet packet { 0 };
    packet.status.packetNumber = packetNumber;
    if (transmitter) {
		if (onTransmit != nullptr) {
			onTransmit(packet);
		}
    } else {
		if (onTransmitTelemetry != nullptr) {
			onTransmitTelemetry(packet);
		}
    }

    rfModule->send((uint8_t *)&packet, sizeof(Packet));

//    HAL_GPIO_WritePin(SYNC_GPIO_Port, SYNC_Pin, GPIO_PIN_RESET);
}

void RfLink::enterRx(void) {
//	HAL_GPIO_WritePin(SYNC_GPIO_Port, SYNC_Pin, GPIO_PIN_SET);

	rf1RxEnable.high();
	rf2RxEnable.high();
    rf1Module->enterRx();
    rf2Module->enterRx();

//    HAL_GPIO_WritePin(SYNC_GPIO_Port, SYNC_Pin, GPIO_PIN_RESET);
}

void RfLink::setPacketParams(bool telemetryPacket) {
	if (telemetryPacket) {
		setNormalPacketParams(rf1Module);
		setNormalPacketParams(rf2Module);
	} else {
		setTelemetryPacketParams(rf1Module, 21);
		setTelemetryPacketParams(rf2Module, 21);
	}
}

void RfLink::setNormalPacketParams(SX1280 *rfModule) {
    PacketParams_t packetParams;
    packetParams.PacketType = PACKET_TYPE_FLRC;
    packetParams.Params.Flrc.PreambleLength = PREAMBLE_LENGTH_32_BITS;
    packetParams.Params.Flrc.SyncWordLength = FLRC_SYNCWORD_LENGTH_4_BYTE;
    packetParams.Params.Flrc.SyncWordMatch = RADIO_RX_MATCH_SYNCWORD_1;
    packetParams.Params.Flrc.HeaderType = RADIO_PACKET_FIXED_LENGTH;
    packetParams.Params.Flrc.PayloadLength = sizeof(Packet);
    packetParams.Params.Flrc.CrcLength = RADIO_CRC_2_BYTES;
    packetParams.Params.Flrc.Whitening = RADIO_WHITENING_OFF;

    rfModule->setPacketParams(&packetParams);
}

void RfLink::setTelemetryPacketParams(SX1280 *rfModule, uint8_t length) {
    PacketParams_t packetParams;
    packetParams.PacketType = PACKET_TYPE_FLRC;
    packetParams.Params.Flrc.PreambleLength = PREAMBLE_LENGTH_32_BITS;
    packetParams.Params.Flrc.SyncWordLength = FLRC_SYNCWORD_LENGTH_4_BYTE;
    packetParams.Params.Flrc.SyncWordMatch = RADIO_RX_MATCH_SYNCWORD_1;
    packetParams.Params.Flrc.HeaderType = RADIO_PACKET_VARIABLE_LENGTH;
    packetParams.Params.Flrc.PayloadLength = length;
    packetParams.Params.Flrc.CrcLength = RADIO_CRC_2_BYTES;
    packetParams.Params.Flrc.Whitening = RADIO_WHITENING_OFF;

    rfModule->setPacketParams(&packetParams);
}

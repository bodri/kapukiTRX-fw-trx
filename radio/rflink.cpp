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
#include "telemetry.h"
#include "radio/transmittersensor.h"
#include "radio/receiversensor.h"

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

		if (packetToSend != nullptr) {
			delete packetToSend;
		}

		// sendPacket xxx us, enterRx: 153 us
		state = SET_PACKET_PARAMS;
		HAL_GPIO_WritePin(SYNC_GPIO_Port, SYNC_Pin, GPIO_PIN_RESET);
	}
		break;
	case SET_PACKET_PARAMS: {
		bool shouldSend = shouldSendPacket();
		bool telemetryPacket = (transmitter && !shouldSend) || (!transmitter && shouldSend);
		packetToSend = new Packet();
		packetToSend->status.packetNumber = packetNumber;
		packetToSend->status.packetType = telemetryPacket ? TELEMETRY : OPENTX;
		packetToSend->size = telemetryPacket ? nextTelemetryPacketSize : 24; // TODO: for normal packets based on number of channels; for telemetry based on telemetry data points
		// TODO: optimize: if no change in packet type, we should not set the same again
		setPacketParams(telemetryPacket);
		state = shouldSend ? WAITING_FOR_TX_OFFSET : ENTER_RX;
	}
		break;
	case WAITING_FOR_TX_OFFSET:
		if (__HAL_TIM_GET_COUNTER(heartBeatTimer) > txOffsetInMicroSecond) {
			sendPacket(packetToSend);
			state = IDLE;
		}
		break;
	case ENTER_RX:
		expectedPacketNumber++;
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
			if (!transmitter) {
				useRf1 = true;
			}
		} else if (module2Received && loadReceivedPacket(rf2Module)) {
			if (!transmitter) {
				useRf1 = false;
			}
		}
		state = DONE;
	}
		break;
	case TIMEOUT:
		state = DONE;
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

		// RSSI
		if (packetNumber % 50 == 0) {
			if (rssiAverageCounter > 0) {
				rf1Rssi = rssi1Sum / rssiAverageCounter;
				rf2Rssi = rssi2Sum / rssiAverageCounter;
			} else {
				rf1Rssi = rf2Rssi = 0;
			}
			rssi1Sum = rssi2Sum = 0;
			rssiAverageCounter = 0;
		}

		// Link Quality
		if (packetNumber % 100 == 0) { // 80 tx packets + 20 rx packets
			if (receiverPacketCounter > 0) {
				linkQuality = expectedPacketNumber / receiverPacketCounter * 100;
			} else {
				linkQuality = 0;
			}
			expectedPacketNumber = 0;
			receiverPacketCounter = 0;
		}

		if (onPrepareTelemetryPacket != nullptr) {
			nextTelemetryPacketSize = onPrepareTelemetryPacket();
		}

		HAL_GPIO_WritePin(SYNC_GPIO_Port, SYNC_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(TXORRX_GPIO_Port, TXORRX_Pin, GPIO_PIN_RESET);
		rf1Module->standBy();
		rf2Module->standBy();
		state = WAITING_FOR_NEXT_HOP;

		// TODO: Can sleep here
	}
		break;
	default:
		break;
	}
}

void RfLink::setTelemetry(Telemetry *telemetry) {
	Sensor *sensor = telemetry->getSensor(transmitter ? transmitterSensor : receiverSensor);
	if (!sensor) { return; }

	if (transmitter) {
		sensor->getTelemetryDataAt(TransmitterSensor::SensorData::rssi1)->setValue(rf1Rssi);
		sensor->getTelemetryDataAt(TransmitterSensor::SensorData::rssi2)->setValue(rf2Rssi);
		sensor->getTelemetryDataAt(TransmitterSensor::SensorData::linkQuality)->setValue(linkQuality);
	} else {
		sensor->getTelemetryDataAt(ReceiverSensor::SensorData::rssi1)->setValue(rf1Rssi);
		sensor->getTelemetryDataAt(ReceiverSensor::SensorData::rssi2)->setValue(rf2Rssi);
		sensor->getTelemetryDataAt(ReceiverSensor::SensorData::linkQuality)->setValue(linkQuality);
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
	Packet packet;
	uint8_t size;

	rfModule->getPayload((uint8_t *)&packet.status, &size, sizeof(packet.payload));

	packet.size = size;
	if (transmitter) {
		rssi1Sum += rf1Module->getRssi(); // RX1 RSSI
		rssi2Sum += rf2Module->getRssi(); // RX2 RSSI
		rssiAverageCounter++;
		receiverPacketCounter++;

		if (onReceiveTelemetry != nullptr) {
			onReceiveTelemetry(packet);
			return true;
		}
	} else {
		this->packetNumber = packet.status.packetNumber; // sync packetNumber with TX

		rssi1Sum += rf1Module->getRssi(); // TX1 RSSI
		rssi2Sum += rf2Module->getRssi(); // TX2 RSSI
		rssiAverageCounter++;
		receiverPacketCounter++;

		if (onReceive != nullptr) {
			onReceive(packet);
			return true;
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

void RfLink::sendPacket(Packet *packet) {
//	HAL_GPIO_WritePin(SYNC_GPIO_Port, SYNC_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(TXORRX_GPIO_Port, TXORRX_Pin, GPIO_PIN_SET);

	useRf1 = transmitter ? !useRf1 : useRf1; // for transmitter alternate modules, receiver uses the same module which received the last packet
	SX1280 *rfModule = useRf1 ? rf1Module : rf2Module;
	if (useRf1) {
		rf2Module->standBy();
		rf1TxEnable.high();
	} else {
		rf1Module->standBy();
		rf2TxEnable.high();
	}

    if (transmitter) {
		if (onTransmit != nullptr) {
			onTransmit(*packet);
		}
    } else {
		if (onTransmitTelemetry != nullptr) {
			onTransmitTelemetry(*packet);
		}
    }

    rfModule->send((uint8_t *)&packet->status, packet->size);

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
	if (packetToSend == nullptr) {
		return;
	}

	if (telemetryPacket) {
		setTelemetryPacketParams(rf1Module, packetToSend->size);
		setTelemetryPacketParams(rf2Module, packetToSend->size);
	} else {
		setNormalPacketParams(rf1Module, packetToSend->size);
		setNormalPacketParams(rf2Module, packetToSend->size);
	}
}

void RfLink::setNormalPacketParams(SX1280 *rfModule, uint8_t length) {
    PacketParams_t packetParams;
    packetParams.PacketType = PACKET_TYPE_FLRC;
    packetParams.Params.Flrc.PreambleLength = PREAMBLE_LENGTH_32_BITS;
    packetParams.Params.Flrc.SyncWordLength = FLRC_SYNCWORD_LENGTH_4_BYTE;
    packetParams.Params.Flrc.SyncWordMatch = RADIO_RX_MATCH_SYNCWORD_1;
    packetParams.Params.Flrc.HeaderType = RADIO_PACKET_FIXED_LENGTH;
    packetParams.Params.Flrc.PayloadLength = length;
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

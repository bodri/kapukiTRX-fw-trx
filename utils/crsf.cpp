/**
 * @file crsf.cpp
 * @brief Handle Crossfire protocol.
 *
 * @author Varadi, Gyorgy, aka bodri
 * Contact: bodri@bodrico.com
 *
 * @bug No known bugs.
 *
 * MIT license, all text above must be included in any redistribution
 *
 */

#include "crsf.h"
#include "radio/transmittersensor.h"
#include "radio/receiversensor.h"

#include "usart.h"
#include "crc.h"

#include <algorithm>

Crossfire::Crossfire(UART_HandleTypeDef *serialPort, CRC_HandleTypeDef *crc): serialPort(serialPort), crc(crc) {
	__HAL_UART_ENABLE_IT(serialPort, UART_IT_IDLE);
//	HAL_UART_Receive_DMA(serialPort, crsfBuffer, sizeof(crsfBuffer));
}

void Crossfire::processSerialError(UART_HandleTypeDef *huart) {
	if (huart->Instance != serialPort->Instance) {
		return;
	}

	__HAL_UART_CLEAR_OREFLAG(huart);
	__HAL_UART_CLEAR_PEFLAG(huart);
	__HAL_UART_CLEAR_NEFLAG(huart);
	__HAL_UART_CLEAR_FEFLAG(huart);
	__HAL_UART_DISABLE_IT(huart, UART_IT_ERR);
	frameError = true;
}

void Crossfire::decodePacket(uint8_t *buffer, size_t maxBufferLength, ChannelData &channelData, volatile bool *packetReceived) {
	  if (frameError) {
		  HAL_UART_Receive_DMA(serialPort, buffer, maxBufferLength);
		  frameError = false;
	  }

	  if (*packetReceived) {
		  size_t maxLength = std::min(sizeof(serialBuffer), maxBufferLength);
		  memcpy(serialBuffer, buffer, maxLength);

		  size_t payloadLength = serialBuffer[1];
		  if (payloadLength > 0 && payloadLength + 1 < maxLength) {
			  uint8_t calculateCrc = HAL_CRC_Calculate(crc, (uint32_t *)&serialBuffer[2], payloadLength - 1);
			  if (serialBuffer[payloadLength + 1] == calculateCrc) {
				  // valid packet

				  if (serialBuffer[0] == 0xEE) {
					  if (serialBuffer[2] == 0x16) {
						  // channel data
//						  uint16_t multiplier = (packet.status.packetType == NORMAL ? 2U : 1U);
						  decode11BitChannels((const uint8_t *)(&serialBuffer[3]), CRSF_MAX_CHANNELS, channelData, 1U, 1U, 0U);

						  sendBackTelemetry();
					  }
				  } else if (serialBuffer[0] == 0xC8) {
					  if (serialBuffer[2] == 0x32 && serialBuffer[6] == 0x05) {
						  // get receiver id
						  uint8_t receiverId = serialBuffer[7];
					  }
				  }
			  }
		  }

		  HAL_UART_Receive_DMA(serialPort, buffer, maxBufferLength);
		  *packetReceived = false;
	  }
}

void Crossfire::decode11BitChannels(const uint8_t *data, uint8_t numberOfChannels, ChannelData &channelData, uint16_t mult, uint16_t div, uint16_t offset) {
#define CHANNEL_SCALE(x) ((int32_t(x) * mult) / div + offset)

    const Channels11Bit *channels = reinterpret_cast<const Channels11Bit *>(data);
    *channelData[0] = CHANNEL_SCALE(channels->ch0);
    *channelData[1] = CHANNEL_SCALE(channels->ch1);
    *channelData[2] = CHANNEL_SCALE(channels->ch2);
    *channelData[3] = CHANNEL_SCALE(channels->ch3);
    *channelData[4] = CHANNEL_SCALE(channels->ch4);
    *channelData[5] = CHANNEL_SCALE(channels->ch5);
    *channelData[6] = CHANNEL_SCALE(channels->ch6);
    *channelData[7] = CHANNEL_SCALE(channels->ch7);
    *channelData[8] = CHANNEL_SCALE(channels->ch8);
    *channelData[9] = CHANNEL_SCALE(channels->ch9);
    *channelData[10] = CHANNEL_SCALE(channels->ch10);
    *channelData[11] = CHANNEL_SCALE(channels->ch11);
    *channelData[12] = CHANNEL_SCALE(channels->ch12);
    *channelData[13] = CHANNEL_SCALE(channels->ch13);
    *channelData[14] = CHANNEL_SCALE(channels->ch14);
    *channelData[15] = CHANNEL_SCALE(channels->ch15);
}

//inline void Crossfire::scaleChannel(uint16_t mult, uint16_t div, uint16_t offset) {
//	return ((int32_t(x) * mult) / div + offset)
//}

void Crossfire::sendBackTelemetry() {
	if (!tracking) {
		return;
	}

	switch (telemetryTypeCounter) {
	case 0:
		sendLinkStatistics();
		break;
	default:
		break;
	}

	telemetryTypeCounter++;
	if (telemetryTypeCounter > 4) {
		telemetryTypeCounter = 0;
	}
}

void Crossfire::sendLinkStatistics() {
	Sensor *txSensor = telemetry->getSensor(transmitterSensor);
	if (!txSensor) { return; }

	uint8_t txRssi1 = txSensor->getTelemetryDataAt(TransmitterSensor::SensorData::rssi1)->getValue();
	uint8_t txRssi2 = txSensor->getTelemetryDataAt(TransmitterSensor::SensorData::rssi2)->getValue();

	uint8_t rxRssi1 { 0 };
	uint8_t rxRssi2 { 0 };
	uint8_t linkQuality { 0 };

	Sensor *rxSensor = telemetry->getSensor(receiverSensor);
	if (rxSensor) {
		rxRssi1 = rxSensor->getTelemetryDataAt(ReceiverSensor::SensorData::rssi1)->getValue();
		rxRssi2 = rxSensor->getTelemetryDataAt(ReceiverSensor::SensorData::rssi2)->getValue();
		linkQuality = rxSensor->getTelemetryDataAt(ReceiverSensor::SensorData::linkQuality)->getValue();
	}

	linkStatistics.rxRssi1 = rxRssi1;
	linkStatistics.rxRssi2 = rxRssi2;
	linkStatistics.rxQuality = linkQuality;
	linkStatistics.rxSnr = 100 + std::max(rxRssi1, rxRssi2); // Calculation for OpenTX RSSI
	linkStatistics.antenna = 1;
	linkStatistics.rfMode = 2;
	linkStatistics.txPower = 3;
	linkStatistics.txRssi = std::max(txRssi1, txRssi2);
	linkStatistics.txQuality = txSensor->getTelemetryDataAt(TransmitterSensor::SensorData::linkQuality)->getValue();
	linkStatistics.txSnr = 100 + linkStatistics.txRssi;  // Calculation for OpenTX RSSI

	size_t len = sizeof(LinkStatisticsFrame);
	telemetryBuffer[0] = 0xEA;
	telemetryBuffer[1] = len + 1;
	memcpy(&telemetryBuffer[2], &linkStatistics, len);

	telemetryBuffer[len + 2] = HAL_CRC_Calculate(crc, (uint32_t*)&telemetryBuffer[2], len);

	HAL_UART_Transmit_DMA(serialPort, telemetryBuffer, len + 3);
}


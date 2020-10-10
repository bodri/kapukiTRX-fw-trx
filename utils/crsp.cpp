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

#include "usart.h"
#include "crc.h"

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
						  decode11BitChannels((const uint8_t *)(&serialBuffer[3]), CRSF_MAX_CHANNELS, channelData, 2U, 1U, 0U);

						  sendBackTelemetry();
					  }
				  } else if (serialBuffer[0] == 0xC8) {
					  if (serialBuffer[2] == 0x32 && serialBuffer[6] == 0x05) {
						  // get receiver id
						  uint8_t receiverId = serialBuffer[7];
						  for (int i = 0; i < 10; i++) { }
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

    const Crossfire::Channels11Bit *channels = (const Channels11Bit *)data;
    channelData[0]->value = CHANNEL_SCALE(channels->ch0);
    channelData[1]->value = CHANNEL_SCALE(channels->ch1);
    channelData[2]->value = CHANNEL_SCALE(channels->ch2);
    channelData[3]->value = CHANNEL_SCALE(channels->ch3);
    channelData[4]->value = CHANNEL_SCALE(channels->ch4);
    channelData[5]->value = CHANNEL_SCALE(channels->ch5);
    channelData[6]->value = CHANNEL_SCALE(channels->ch6);
    channelData[7]->value = CHANNEL_SCALE(channels->ch7);
    channelData[8]->value = CHANNEL_SCALE(channels->ch8);
    channelData[9]->value = CHANNEL_SCALE(channels->ch9);
    channelData[10]->value = CHANNEL_SCALE(channels->ch10);
    channelData[11]->value = CHANNEL_SCALE(channels->ch11);
    channelData[12]->value = CHANNEL_SCALE(channels->ch12);
    channelData[13]->value = CHANNEL_SCALE(channels->ch13);
    channelData[14]->value = CHANNEL_SCALE(channels->ch14);
    channelData[15]->value = CHANNEL_SCALE(channels->ch15);
}

//inline void Crossfire::scaleChannel(uint16_t mult, uint16_t div, uint16_t offset) {
//	return ((int32_t(x) * mult) / div + offset)
//}

void Crossfire::sendBackTelemetry() {
	sendLinkStatistics();
}

void Crossfire::sendLinkStatistics() {
	linkStatistics.rxRssi1 = 78;
	linkStatistics.rxRssi2 = 99;
	linkStatistics.rxQuality = 34;
	linkStatistics.rxSnr = 22;
	linkStatistics.antenna = 1;
	linkStatistics.rfMode = 0;
	linkStatistics.txPower = 3;
	linkStatistics.txRssi = 77;
	linkStatistics.txQuality = 56;
	linkStatistics.txSnr = 12;

	size_t len = sizeof(LinkStatisticsFrame);
	telemetryBuffer[0] = 0xEA;
	telemetryBuffer[1] = len + 1;
	memcpy(&telemetryBuffer[2], &linkStatistics, len);

	telemetryBuffer[len + 2] = HAL_CRC_Calculate(crc, (uint32_t*)&telemetryBuffer[2], len);

	HAL_UART_Transmit_DMA(serialPort, telemetryBuffer, len + 3);
}


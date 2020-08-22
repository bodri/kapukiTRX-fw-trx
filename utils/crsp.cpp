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

void Crossfire::decodePacket(uint8_t *buffer, size_t maxBufferLength, ChannelData &channelData) {
	  if (frameError) {
		  HAL_UART_Receive_DMA(&huart3, serialBuffer, sizeof(serialBuffer));
		  frameError = false;
	  }

	  if (packetReceived) {
		  memcpy(serialBuffer, buffer, std::min(sizeof(serialBuffer), maxBufferLength));

		  if (serialBuffer[0] == 0xEE) {
			  uint8_t len = serialBuffer[1] - 1;
			  uint8_t crc = HAL_CRC_Calculate(&hcrc, (uint32_t *)&serialBuffer[2], len);
			  if (serialBuffer[len + 2] == crc) {
				  // valid packet
				  decode11BitChannels((const uint8_t *)(&serialBuffer[3]), CRSF_MAX_CHANNELS, channelData, 2U, 1U, 0U);

				  // send back dummy telemetry
				  uint8_t telemetry[] = "\x14\xA2\xA2\x55\x65\x02\x00\x05\x96\x44\x22";
				  static uint8_t buffer[sizeof(telemetry) + 2];
				  size_t len = sizeof(telemetry) - 1;
				  memcpy(&buffer[2], &telemetry[0], len);
				  buffer[0] = 0xEA;
				  buffer[1] = len + 1;
				  buffer[sizeof(buffer) - 1] = HAL_CRC_Calculate(&hcrc, (uint32_t *)&telemetry[0], len);
				  for (int i = 0; i < 1000; i++) { }
				  HAL_UART_Transmit_DMA(&huart3, buffer, sizeof(buffer));
			  } else {
				  for (int i = 0; i < 100; i++) { }
			  }
		  }

		  HAL_UART_Receive_DMA(&huart3, serialBuffer, sizeof(serialBuffer));
		  packetReceived = false;
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


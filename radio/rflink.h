/**
 * @file rflink.h
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

#ifndef __RFLINK_H__
#define __RFLINK_H__

#include "main.h"
#include "patterngen.h"
#include "sx1280.h"

#include <stdint.h>
#include <map>

class Telemetry;

typedef enum {
	NORMAL = 1,
	OPENTX,
	TELEMETRY
} PacketType;

typedef struct __attribute__ ((__packed__)) {
	uint8_t size;
	struct {
		uint16_t packetType : 3;
		uint16_t rfu : 3;
		uint16_t packetNumber : 10;
	} status;

	uint8_t payload[127];

	struct NormalChannels {
	// 192 bits of data (12 bits per channel * 16 channels) = 24 bytes.
	#if __BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__
	#error "Only supported on little-endian architectures"
	#endif
		uint16_t ch0 : 12;
		uint16_t ch1 : 12;
		uint16_t ch2 : 12;
		uint16_t ch3 : 12;
		uint16_t ch4 : 12;
		uint16_t ch5 : 12;
		uint16_t ch6 : 12;
		uint16_t ch7 : 12;
		uint16_t ch8 : 12;
		uint16_t ch9 : 12;
		uint16_t ch10 : 12;
		uint16_t ch11 : 12;
		uint16_t ch12 : 12;
		uint16_t ch13 : 12;
		uint16_t ch14 : 12;
		uint16_t ch15 : 12;
	} __attribute__ ((__packed__));

	struct OpenTxChannels {
	// 176 bits of data (11 bits per channel * 16 channels) = 22 bytes.
	#if __BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__
	#error "Only supported on little-endian architectures"
	#endif
		uint16_t ch0 : 11;
		uint16_t ch1 : 11;
		uint16_t ch2 : 11;
		uint16_t ch3 : 11;
		uint16_t ch4 : 11;
		uint16_t ch5 : 11;
		uint16_t ch6 : 11;
		uint16_t ch7 : 11;
		uint16_t ch8 : 11;
		uint16_t ch9 : 11;
		uint16_t ch10 : 11;
		uint16_t ch11 : 11;
		uint16_t ch12 : 11;
		uint16_t ch13 : 11;
		uint16_t ch14 : 11;
		uint16_t ch15 : 11;
	} __attribute__ ((__packed__));

} Packet;

typedef enum {
					INIT,
					START,
					SET_PACKET_PARAMS,
	// Tx							// Rx
	WAITING_FOR_TX_OFFSET,			ENTER_RX,
					IDLE,
	TRANSMITTED,					RECEIVED,
					TIMEOUT,
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
	void setTelemetry(Telemetry *telemetry);

	std::function<void(Packet &packet)> onTransmit;
	std::function<uint8_t()> onPrepareTelemetryPacket;
	std::function<void(Packet &packet)> onTransmitTelemetry;
	std::function<void(Packet &packet)> onReceive;
	std::function<void(Packet &packet)> onReceiveTelemetry;
	std::function<void(bool tracking)> onLinkStatusChange;

private:
	TIM_HandleTypeDef *heartBeatTimer;

	SX1280 *rf1Module;
	SX1280 *rf2Module;

	bool transmitter;
	bool tracking { false };

	PatternGenerator *patternGenerator;

	Pin rf1TxEnable { Pin(RF1TXEN_GPIO_Port, RF1TXEN_Pin) };
	Pin rf1RxEnable { Pin(RF1RXEN_GPIO_Port, RF1RXEN_Pin) };

	Pin rf2TxEnable { Pin(RF2TXEN_GPIO_Port, RF2TXEN_Pin) };
	Pin rf2RxEnable { Pin(RF2RXEN_GPIO_Port, RF2RXEN_Pin) };

	volatile LinkState state { INIT };
	Packet *packetToSend { nullptr };
	uint16_t packetNumber;
	uint8_t nextTelemetryPacketSize { 127 };

	volatile bool heartBeatTimeout { false };
	volatile IrqSource lastIrqSource { NO_IRQ };
	bool useRf1 { false };
	uint32_t lostPacket { 0 };
	std::map<uint8_t, int> failuresPerChannel { };
	uint16_t timerWhenSyncReceived { 3255 }; // = transmit time (2255us) + Tx offset (1000us)
	uint16_t txOffsetInMicroSecond { 1000 };

	const uint8_t lostPacketTreshold { 20 };
	const uint8_t downLinkFrequency { 4 };

	int32_t rssi1Sum { 0 };
	int32_t rssi2Sum { 0 };
	int32_t rssiAverageCounter { 0 };
	int8_t rf1Rssi { 0 };
	int8_t rf2Rssi { 0 };
	uint32_t expectedPacketNumber { 0 };
	uint32_t receiverPacketCounter { 0 };
	uint8_t linkQuality { 0 };

	bool validPacket(SX1280 *rfModule);
	bool loadReceivedPacket(SX1280 *rfModule);
	bool shouldSendPacket(void);
	void setTracking(bool tracking);
	void adjustTimerToTrackTx(void);
	void registerLostPacket(void);
	void sendPacket(Packet *packet);
	void enterRx(void);

	void setPacketParams(bool telemetryPacket);
	void setNormalPacketParams(SX1280 *rfModule, uint8_t length);
	void setTelemetryPacketParams(SX1280 *rfModule, uint8_t length);
};

#endif // __RFLINK_H__

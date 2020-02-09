/*
  ______                              _
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2016 Semtech

Description: Driver for SX1280 devices

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis, Gregory Cristian and Matthieu Verdy
*/
#ifndef __SX1280_H__
#define __SX1280_H__

#include <pin.hpp>
#include "main.h"
#include "sx1280hw.h"
#include "rfmodule.h"
#include <cmath>
#include <cstring>

class SX1280: public RfModule {
public:
    SX1280(SPI_HandleTypeDef *spi,
    		Pin nresetPin,
    		Pin ncsPin,
    		Pin busyPin):
		radioSpi(spi),
		nresetPin(nresetPin),
		ncsPin(ncsPin),
		busyPin(busyPin),
		operatingMode( MODE_STDBY_RC ),
		packetType(PACKET_TYPE_NONE)
    { }

    virtual ~SX1280() { }

    void init(void);
    void standBy(void);
    void setAddress(uint16_t address);
    void setChannel(uint8_t channel);
	void enterRx(void);
	void send(uint8_t *payload, uint8_t size);

    uint16_t getFirmwareVersion(void);
    void setRegulatorMode(RadioRegulatorModes_t mode);

    void setPacketType(RadioPacketTypes_t packetType);
    void setPacketParams(PacketParams_t *packetParams);
    void getPacketStatus(PacketStatus_t *packetStatus);
    int8_t getRssi(void);
    void getRxBufferStatus(uint8_t *rxPayloadLength, uint8_t *rxStartBufferPointer);
    void setModulationParams(ModulationParams_t *modParams);
    RadioPacketTypes_t getPacketType(bool returnLocalCopy = false);
    void setRfFrequency(uint32_t rfFrequency);
    void setTxParams(int8_t power, RadioRampTimes_t rampTime);
    void setBufferBaseAddresses(uint8_t txBaseAddress, uint8_t rxBaseAddress);
    void setDioIrqParams(uint16_t irqMask, uint16_t dio1Mask, uint16_t dio2Mask, uint16_t dio3Mask);
    uint16_t getIrqStatus(void);
    void clearIrqStatus(uint16_t irqMask);

    RadioStatus_t getStatus(void);
    void wakeup(void);
    void setSleep(SleepParams_t sleepConfig);
    void setStandBy(RadioStandbyModes_t mode);
    void setRx(TickTime_t timeout);
    void setTx(TickTime_t timeout);
    void setPayload(uint8_t *payload, uint8_t size, uint8_t offset = 0x00);
    bool getPayload(uint8_t *payload, uint8_t *size, uint8_t maxSize);
    void sendPayload(uint8_t *payload, uint8_t size, TickTime_t timeout, uint8_t offset = 0x00);
    bool setSyncWord(uint8_t syncWordIdx, uint8_t *syncWord);
    void setSyncWordErrorTolerance(uint8_t errorBits);
    bool setCrcSeed(uint8_t *seed);
    void setCrcPolynomial(uint16_t polynomial);
    void setLNAGainSetting(const RadioLnaSettings_t lnaSetting);
    void setAutoFs(const bool autoFs);

    void processIrqs(void);

private:
    inline void waitOnBusy(void);
    void readCommand(RadioCommands_t opcode, uint8_t *buffer, uint16_t size);
    void writeCommand(RadioCommands_t command, uint8_t *buffer, uint16_t size);
    void readRegister(uint16_t address, uint8_t *buffer, uint16_t size);
    uint8_t readRegister(uint16_t address);
    void writeRegister(uint16_t address, uint8_t *buffer, uint16_t size);
    void writeRegister(uint16_t address, uint8_t value);
    void readBuffer(uint8_t offset, uint8_t *buffer, uint8_t size);
    void writeBuffer(uint8_t offset, uint8_t *buffer, uint8_t size);
    void resetChip(void);

protected:
    SPI_HandleTypeDef *radioSpi;
    Pin nresetPin;
    Pin ncsPin;
    Pin busyPin;

    RadioOperatingModes_t operatingMode;
    RadioPacketTypes_t packetType;

    static const std::array<uint32_t, 83> frequencyLookUpTable;
};

#endif // __SX1280_H__

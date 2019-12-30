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

#include "sx1280.h"

#include <algorithm>

using namespace std;

void SX1280::init(void) {
    resetChip();
    wakeup();
	setRegulatorMode(USE_DCDC);

    ModulationParams_t modulationParams;
    modulationParams.PacketType = PACKET_TYPE_FLRC;
    modulationParams.Params.Flrc.BitrateBandwidth = FLRC_BR_0_325_BW_0_3;
    modulationParams.Params.Flrc.CodingRate = FLRC_CR_3_4;
    modulationParams.Params.Flrc.ModulationShaping = RADIO_MOD_SHAPING_BT_OFF;

    PacketParams_t packetParams;
    packetParams.PacketType = PACKET_TYPE_FLRC;
    packetParams.Params.Flrc.PreambleLength = PREAMBLE_LENGTH_32_BITS;
    packetParams.Params.Flrc.SyncWordLength = FLRC_SYNCWORD_LENGTH_4_BYTE;
    packetParams.Params.Flrc.SyncWordMatch = RADIO_RX_MATCH_SYNCWORD_1;
    packetParams.Params.Flrc.HeaderType = RADIO_PACKET_FIXED_LENGTH;
    packetParams.Params.Flrc.PayloadLength = 41; //BUFFER_SIZE;
    packetParams.Params.Flrc.CrcLength = RADIO_CRC_2_BYTES;
    packetParams.Params.Flrc.Whitening = RADIO_WHITENING_OFF;

    setStandBy(STDBY_RC);
    setPacketType(modulationParams.PacketType);
    setModulationParams(&modulationParams);
    setPacketParams(&packetParams);
    setBufferBaseAddresses(0x00, 0x00);
    setTxParams(0, RADIO_RAMP_02_US); //TX_OUTPUT_POWER
    setLNAGainSetting(LNA_HIGH_SENSITIVITY_MODE);
    setAutoFs();
}

void SX1280::standBy(void) {
	setStandBy(STDBY_RC);
}

void SX1280::setAddress(uint16_t address) {
    std::array<uint8_t, 4> syncWord { 0xB2, (uint8_t)(address >> 8), (uint8_t)address, 0 };
    setSyncWord(1, syncWord.data());
}

void SX1280::setChannel(uint8_t channel) {
	size_t index = std::min((size_t)channel, SX1280::frequencyLookUpTable.size() - 1);
	setRfFrequency(SX1280::frequencyLookUpTable[index]);
}

void SX1280::enterRx(void) {
	TickTime_t timeout { RADIO_TICK_SIZE_1000_US,  0 };
	setRx(timeout);
}

void SX1280::send(uint8_t *payload, uint8_t size) {
	TickTime_t timeout { RADIO_TICK_SIZE_1000_US,  0 };
	sendPayload(payload, size, timeout, 0);
}

uint16_t SX1280::getFirmwareVersion(void) {
	uint8_t buf[2];
	readRegister(REG_LR_FIRMWARE_VERSION_MSB, buf, sizeof(buf));
    return buf[0] << 8 | buf[1];
}

void SX1280::setRegulatorMode(RadioRegulatorModes_t mode) {
    writeCommand(RADIO_SET_REGULATORMODE, (uint8_t *)&mode, 1);
}

void SX1280::setPacketType(RadioPacketTypes_t packetType) {
    this->packetType = packetType;
    writeCommand(RADIO_SET_PACKETTYPE, (uint8_t *)&packetType, 1);
}

void SX1280::setPacketParams(PacketParams_t *packetParams) {
    uint8_t buf[7];

    if( this->packetType != packetParams->PacketType) {
        this->setPacketType( packetParams->PacketType);
    }

    memset(buf, 0, sizeof(buf));
    switch (packetParams->PacketType) {
        case PACKET_TYPE_GFSK:
            buf[0] = packetParams->Params.Gfsk.PreambleLength;
            buf[1] = packetParams->Params.Gfsk.SyncWordLength;
            buf[2] = packetParams->Params.Gfsk.SyncWordMatch;
            buf[3] = packetParams->Params.Gfsk.HeaderType;
            buf[4] = packetParams->Params.Gfsk.PayloadLength;
            buf[5] = packetParams->Params.Gfsk.CrcLength;
            buf[6] = packetParams->Params.Gfsk.Whitening;
            break;
        case PACKET_TYPE_LORA:
        case PACKET_TYPE_RANGING:
            buf[0] = packetParams->Params.LoRa.PreambleLength;
            buf[1] = packetParams->Params.LoRa.HeaderType;
            buf[2] = packetParams->Params.LoRa.PayloadLength;
            buf[3] = packetParams->Params.LoRa.Crc;
            buf[4] = packetParams->Params.LoRa.InvertIQ;
            break;
        case PACKET_TYPE_FLRC:
            buf[0] = packetParams->Params.Flrc.PreambleLength;
            buf[1] = packetParams->Params.Flrc.SyncWordLength;
            buf[2] = packetParams->Params.Flrc.SyncWordMatch;
            buf[3] = packetParams->Params.Flrc.HeaderType;
            buf[4] = packetParams->Params.Flrc.PayloadLength;
            buf[5] = packetParams->Params.Flrc.CrcLength;
            buf[6] = packetParams->Params.Flrc.Whitening;
            break;
        case PACKET_TYPE_BLE:
            buf[0] = packetParams->Params.Ble.ConnectionState;
            buf[1] = packetParams->Params.Ble.CrcLength;
            buf[2] = packetParams->Params.Ble.BleTestPayload;
            buf[3] = packetParams->Params.Ble.Whitening;
            break;
        case PACKET_TYPE_NONE:
            break;
    }

    writeCommand(RADIO_SET_PACKETPARAMS, buf, sizeof(buf));
}

void SX1280::getPacketStatus(PacketStatus_t *packetStatus) {
    uint8_t status[5];

    readCommand(RADIO_GET_PACKETSTATUS, status, sizeof(status));

    packetStatus->packetType = this->getPacketType(true);
    switch (packetStatus->packetType) {
        case PACKET_TYPE_GFSK:
            packetStatus->Gfsk.RssiAvg = -(status[0] / 2);
            packetStatus->Gfsk.RssiSync = -(status[1] / 2);

            packetStatus->Gfsk.ErrorStatus.SyncError = (status[2] >> 6) & 0x01;
            packetStatus->Gfsk.ErrorStatus.LengthError = (status[2] >> 5) & 0x01;
            packetStatus->Gfsk.ErrorStatus.CrcError = (status[2] >> 4) & 0x01;
            packetStatus->Gfsk.ErrorStatus.AbortError = (status[2] >> 3) & 0x01;
            packetStatus->Gfsk.ErrorStatus.HeaderReceived = (status[2] >> 2) & 0x01;
            packetStatus->Gfsk.ErrorStatus.PacketReceived = (status[2] >> 1) & 0x01;
            packetStatus->Gfsk.ErrorStatus.PacketControlerBusy = status[2] & 0x01;

            packetStatus->Gfsk.TxRxStatus.RxNoAck = (status[3] >> 5) & 0x01;
            packetStatus->Gfsk.TxRxStatus.PacketSent = status[3] & 0x01;

            packetStatus->Gfsk.SyncAddrStatus = status[4] & 0x07;
            break;

        case PACKET_TYPE_LORA:
        case PACKET_TYPE_RANGING:
            packetStatus->LoRa.RssiPkt = -(status[0] / 2);
            (status[1] < 128) ? (packetStatus->LoRa.SnrPkt = status[1] / 4) : (packetStatus->LoRa.SnrPkt = ((status[1] - 256) /4));
            break;

        case PACKET_TYPE_FLRC:
            packetStatus->Flrc.RssiAvg = -(status[0] / 2);
            packetStatus->Flrc.RssiSync = -(status[1] / 2);

            packetStatus->Flrc.ErrorStatus.SyncError = (status[2] >> 6) & 0x01;
            packetStatus->Flrc.ErrorStatus.LengthError = (status[2] >> 5) & 0x01;
            packetStatus->Flrc.ErrorStatus.CrcError = (status[2] >> 4) & 0x01;
            packetStatus->Flrc.ErrorStatus.AbortError = (status[2] >> 3) & 0x01;
            packetStatus->Flrc.ErrorStatus.HeaderReceived = (status[2] >> 2) & 0x01;
            packetStatus->Flrc.ErrorStatus.PacketReceived = (status[2] >> 1) & 0x01;
            packetStatus->Flrc.ErrorStatus.PacketControlerBusy = status[2] & 0x01;

            packetStatus->Flrc.TxRxStatus.RxPid = (status[3] >> 6) & 0x03;
            packetStatus->Flrc.TxRxStatus.RxNoAck = (status[3] >> 5) & 0x01;
            packetStatus->Flrc.TxRxStatus.RxPidErr = (status[3] >> 4) & 0x01;
            packetStatus->Flrc.TxRxStatus.PacketSent = status[3] & 0x01;

            packetStatus->Flrc.SyncAddrStatus = status[4] & 0x07;
            break;

        case PACKET_TYPE_BLE:
            packetStatus->Ble.RssiAvg = -(status[0] / 2);
            packetStatus->Ble.RssiSync =  -(status[1] / 2);

            packetStatus->Ble.ErrorStatus.SyncError = (status[2] >> 6) & 0x01;
            packetStatus->Ble.ErrorStatus.LengthError = (status[2] >> 5) & 0x01;
            packetStatus->Ble.ErrorStatus.CrcError = (status[2] >> 4) & 0x01;
            packetStatus->Ble.ErrorStatus.AbortError = (status[2] >> 3) & 0x01;
            packetStatus->Ble.ErrorStatus.HeaderReceived = (status[2] >> 2) & 0x01;
            packetStatus->Ble.ErrorStatus.PacketReceived = (status[2] >> 1) & 0x01;
            packetStatus->Ble.ErrorStatus.PacketControlerBusy = status[2] & 0x01;

            packetStatus->Ble.TxRxStatus.PacketSent = status[3] & 0x01;

            packetStatus->Ble.SyncAddrStatus = status[4] & 0x07;
            break;

        case PACKET_TYPE_NONE:
            // In that specific case, we set everything in the packetStatus to zeros
            // and reset the packet type accordingly
            memset(packetStatus, 0, sizeof(PacketStatus_t));
            packetStatus->packetType = PACKET_TYPE_NONE;
            break;
    }
}

int8_t SX1280::getRssi(void) {
    uint8_t raw = 0;

    readCommand(RADIO_GET_RSSIINST, &raw, 1);
    return (int8_t)(-raw / 2);
}

void SX1280::getRxBufferStatus(uint8_t *rxPayloadLength, uint8_t *rxStartBufferPointer) {
    uint8_t status[2];

    readCommand(RADIO_GET_RXBUFFERSTATUS, status, sizeof(status));

    // In case of LORA fixed header, the rxPayloadLength is obtained by reading
    // the register REG_LR_PAYLOADLENGTH
    RadioPacketTypes_t packetType = getPacketType(true);
    if ((packetType == PACKET_TYPE_LORA) && (readRegister(REG_LR_PACKETPARAMS) >> 7 == 1)) {
        *rxPayloadLength = readRegister(REG_LR_PAYLOADLENGTH);
    } else if (packetType == PACKET_TYPE_BLE) {
        // In the case of BLE, the size returned in status[0] do not include the 2-byte length PDU header
        // so it is added there
        *rxPayloadLength = status[0] + 2;
    } else {
        *rxPayloadLength = status[0];
    }

    *rxStartBufferPointer = status[1];
}

void SX1280::setModulationParams(ModulationParams_t *modParams) {
    uint8_t buf[3];

    if (this->packetType != modParams->PacketType) {
        this->setPacketType(modParams->PacketType);
    }

    memset(buf, 0, sizeof(buf));
    switch (modParams->PacketType) {
        case PACKET_TYPE_GFSK:
            buf[0] = modParams->Params.Gfsk.BitrateBandwidth;
            buf[1] = modParams->Params.Gfsk.ModulationIndex;
            buf[2] = modParams->Params.Gfsk.ModulationShaping;
            break;
        case PACKET_TYPE_LORA:
        case PACKET_TYPE_RANGING:
            buf[0] = modParams->Params.LoRa.SpreadingFactor;
            buf[1] = modParams->Params.LoRa.Bandwidth;
            buf[2] = modParams->Params.LoRa.CodingRate;
//            this->LoRaBandwidth = modParams->Params.LoRa.Bandwidth;
            break;
        case PACKET_TYPE_FLRC:
            buf[0] = modParams->Params.Flrc.BitrateBandwidth;
            buf[1] = modParams->Params.Flrc.CodingRate;
            buf[2] = modParams->Params.Flrc.ModulationShaping;
            break;
        case PACKET_TYPE_BLE:
            buf[0] = modParams->Params.Ble.BitrateBandwidth;
            buf[1] = modParams->Params.Ble.ModulationIndex;
            buf[2] = modParams->Params.Ble.ModulationShaping;
            break;
        case PACKET_TYPE_NONE:
            break;
    }

    writeCommand(RADIO_SET_MODULATIONPARAMS, buf, sizeof(buf));
}

void SX1280::setRfFrequency(uint32_t rfFrequency) {
    uint8_t buf[3];
    uint32_t freq = 0;

    freq = (uint32_t)((double)rfFrequency / (double)FREQ_STEP);
    buf[0] = (uint8_t)(freq >> 16);
    buf[1] = (uint8_t)(freq >> 8);
    buf[2] = (uint8_t)freq;
    writeCommand(RADIO_SET_RFFREQUENCY, buf, sizeof(buf));
}

RadioPacketTypes_t SX1280::getPacketType(bool returnLocalCopy) {
    RadioPacketTypes_t packetType = PACKET_TYPE_NONE;
    if (returnLocalCopy == false) {
        readCommand(RADIO_GET_PACKETTYPE, (uint8_t *)&packetType, 1);
        if (this->packetType != packetType) {
            this->packetType = packetType;
        }
    } else {
        packetType = this->packetType;
    }

    return packetType;
}

void SX1280::setTxParams(int8_t power, RadioRampTimes_t rampTime) {
    uint8_t buf[2];

    // The power value to send on SPI/UART is in the range [0..31] and the
    // physical output power is in the range [-18..13]dBm
    buf[0] = power + 18;
    buf[1] = (uint8_t)rampTime;
    writeCommand(RADIO_SET_TXPARAMS, buf, sizeof(buf));
}

void SX1280::setBufferBaseAddresses(uint8_t txBaseAddress, uint8_t rxBaseAddress) {
    uint8_t buf[2];

    buf[0] = txBaseAddress;
    buf[1] = rxBaseAddress;
    writeCommand(RADIO_SET_BUFFERBASEADDRESS, buf, sizeof(buf));
}

void SX1280::setDioIrqParams(uint16_t irqMask, uint16_t dio1Mask, uint16_t dio2Mask, uint16_t dio3Mask) {
    uint8_t buf[8];

    buf[0] = (uint8_t)(irqMask >> 8);
    buf[1] = (uint8_t)irqMask;
    buf[2] = (uint8_t)(dio1Mask >> 8);
    buf[3] = (uint8_t)dio1Mask;
    buf[4] = (uint8_t)(dio2Mask >> 8);
    buf[5] = (uint8_t)dio2Mask;
    buf[6] = (uint8_t)(dio3Mask >> 8);
    buf[7] = (uint8_t)dio3Mask;
    writeCommand(RADIO_SET_DIOIRQPARAMS, buf, sizeof(buf));
}

uint16_t SX1280::getIrqStatus(void) {
    uint8_t irqStatus[2];
    readCommand(RADIO_GET_IRQSTATUS, irqStatus, sizeof(irqStatus));
    return (irqStatus[0] << 8) | irqStatus[1];
}

void SX1280::clearIrqStatus(uint16_t irqMask) {
    uint8_t buf[2];

    buf[0] = (uint8_t)(irqMask >> 8);
    buf[1] = (uint8_t)irqMask;
    writeCommand(RADIO_CLR_IRQSTATUS, buf, sizeof(buf));
}

RadioStatus_t SX1280::getStatus(void) {
    uint8_t stat = 0;
    RadioStatus_t status;

    readCommand(RADIO_GET_STATUS, (uint8_t *)&stat, 1);
    status.Value = stat;
    return status;
}

void SX1280::wakeup(void) {
//    __disable_irq( );

    //Don't wait for BUSY here

    if (radioSpi != NULL) {
    	ncsPin.low();
    	uint8_t txBuffer[] = { RADIO_GET_STATUS, 0 };
    	uint8_t rxBuffer[sizeof(txBuffer)];
    	HAL_SPI_TransmitReceive(this->radioSpi, txBuffer, rxBuffer, sizeof(txBuffer), 1000);
    	ncsPin.high();
    }

    // Wait for chip to be ready.
    waitOnBusy( );

//    __enable_irq( );
}

void SX1280::setSleep(SleepParams_t sleepConfig) {
    uint8_t sleep = (sleepConfig.WakeUpRTC << 3) |
                    (sleepConfig.InstructionRamRetention << 2) |
                    (sleepConfig.DataBufferRetention << 1) |
                    (sleepConfig.DataRamRetention);

    operatingMode = MODE_SLEEP;
    writeCommand(RADIO_SET_SLEEP, &sleep, 1);
}

void SX1280::setStandBy(RadioStandbyModes_t standbyConfig) {
    writeCommand(RADIO_SET_STANDBY, (uint8_t *)&standbyConfig, 1);
    if (standbyConfig == STDBY_RC) {
        operatingMode = MODE_STDBY_RC;
    } else {
        operatingMode = MODE_STDBY_XOSC;
    }
}

void SX1280::setRx(TickTime_t timeout) {
    uint8_t buf[3];
    buf[0] = timeout.PeriodBase;
    buf[1] = (uint8_t)(timeout.PeriodBaseCount >> 8);
    buf[2] = (uint8_t)timeout.PeriodBaseCount;

    clearIrqStatus(IRQ_RADIO_ALL);

    // If the radio is doing ranging operations, then apply the specific calls
    // prior to SetRx
//    if (getPacketType(true) == PACKET_TYPE_RANGING) {
//        setRangingRole(RADIO_RANGING_ROLE_SLAVE);
//    }

    writeCommand(RADIO_SET_RX, buf, sizeof(buf));
    operatingMode = MODE_RX;
}

void SX1280::setTx(TickTime_t timeout) {
    uint8_t buf[3];
    buf[0] = timeout.PeriodBase;
    buf[1] = (uint8_t)(timeout.PeriodBaseCount >> 8);
    buf[2] = (uint8_t)timeout.PeriodBaseCount;

    clearIrqStatus(IRQ_RADIO_ALL);

    // If the radio is doing ranging operations, then apply the specific calls
    // prior to SetTx
//    if (getPacketType(true) == PACKET_TYPE_RANGING) {
//        setRangingRole(RADIO_RANGING_ROLE_MASTER);
//    }

    writeCommand(RADIO_SET_TX, buf, sizeof(buf));
    operatingMode = MODE_TX;
}

void SX1280::setPayload(uint8_t *buffer, uint8_t size, uint8_t offset) {
    writeBuffer(offset, buffer, size);
}

bool SX1280::getPayload(uint8_t *buffer, uint8_t *size , uint8_t maxSize) {
    uint8_t offset;

    this->getRxBufferStatus(size, &offset);
    if (*size > maxSize) {
        return false;
    }

    readBuffer(offset, buffer, *size);
    return true;
}

void SX1280::sendPayload(uint8_t *payload, uint8_t size, TickTime_t timeout, uint8_t offset) {
	this->setPayload(payload, size, offset);
	this->setTx(timeout);
}

bool SX1280::setSyncWord(uint8_t syncWordIdx, uint8_t *syncWord) {
    uint16_t addr;
    uint8_t syncwordSize = 0;

    switch (getPacketType(true)) {
        case PACKET_TYPE_GFSK:
            syncwordSize = 5;
            switch (syncWordIdx) {
                case 1:
                    addr = REG_LR_SYNCWORDBASEADDRESS1;
                    break;
                case 2:
                    addr = REG_LR_SYNCWORDBASEADDRESS2;
                    break;
                case 3:
                    addr = REG_LR_SYNCWORDBASEADDRESS3;
                    break;
                default:
                    return false;
            }
            break;
        case PACKET_TYPE_FLRC:
            // For FLRC packet type, the SyncWord is one byte shorter and
            // the base address is shifted by one byte
            syncwordSize = 4;
            switch (syncWordIdx) {
                case 1:
                    addr = REG_LR_SYNCWORDBASEADDRESS1 + 1;
                    break;
                case 2:
                    addr = REG_LR_SYNCWORDBASEADDRESS2 + 1;
                    break;
                case 3:
                    addr = REG_LR_SYNCWORDBASEADDRESS3 + 1;
                    break;
                default:
                    return false;
            }
            break;
        case PACKET_TYPE_BLE:
            // For Ble packet type, only the first SyncWord is used and its
            // address is shifted by one byte
            syncwordSize = 4;
            switch (syncWordIdx) {
                case 1:
                    addr = REG_LR_SYNCWORDBASEADDRESS1 + 1;
                    break;
                default:
                    return false;
            }
            break;
        default:
            return false;
    }

    writeRegister(addr, syncWord, syncwordSize);
    return true;
}

void SX1280::setSyncWordErrorTolerance(uint8_t errorBits) {
    errorBits = (readRegister(REG_LR_SYNCWORDTOLERANCE) & 0xF0) | (errorBits & 0x0F);
    writeRegister(REG_LR_SYNCWORDTOLERANCE, errorBits);
}

bool SX1280::setCrcSeed(uint8_t *seed) {
    uint8_t updated = false;

    switch (getPacketType(true)) {
        case PACKET_TYPE_GFSK:
        case PACKET_TYPE_FLRC:
            writeRegister(REG_LR_CRCSEEDBASEADDR, seed, 2);
            updated = true;
            break;
        case PACKET_TYPE_BLE:
            writeRegister(0x9c7, seed[2]);
            writeRegister(0x9c8, seed[1]);
            writeRegister(0x9c9, seed[0]);
            updated = true;
            break;
        default:
            break;
    }

    return updated;
}

void SX1280::setCrcPolynomial(uint16_t polynomial) {
    uint8_t val[2];

    val[0] = (uint8_t)(polynomial >> 8 );
    val[1] = (uint8_t)polynomial;

    switch (getPacketType(true)) {
        case PACKET_TYPE_GFSK:
        case PACKET_TYPE_FLRC:
            writeRegister(REG_LR_CRCPOLYBASEADDR, val, sizeof(val));
            break;
        default:
            break;
    }
}

void SX1280::setLNAGainSetting(const RadioLnaSettings_t lnaSetting) {
    switch(lnaSetting) {
        case LNA_HIGH_SENSITIVITY_MODE: {
            writeRegister(REG_LNA_REGIME, readRegister(REG_LNA_REGIME) | MASK_LNA_REGIME);
            break;
        }
        case LNA_LOW_POWER_MODE: {
            writeRegister(REG_LNA_REGIME, readRegister(REG_LNA_REGIME) & ~MASK_LNA_REGIME);
            break;
        }
    }
}

void SX1280::setAutoFs() {
    uint8_t buf[1];
    buf[0] = 1;
    writeCommand(RADIO_SET_AUTOFS, buf, sizeof(buf));
}

void SX1280::processIrqs(void) {
    RadioPacketTypes_t packetType = PACKET_TYPE_NONE;

    packetType = getPacketType(true);
    uint16_t irqRegs = getIrqStatus();
    clearIrqStatus(IRQ_RADIO_ALL);

    switch (packetType) {
        case PACKET_TYPE_GFSK:
        case PACKET_TYPE_FLRC:
        case PACKET_TYPE_BLE:
            switch (operatingMode) {
                case MODE_RX:
                    if ((irqRegs & IRQ_RX_DONE) == IRQ_RX_DONE) {
                        if ((irqRegs & IRQ_CRC_ERROR) == IRQ_CRC_ERROR) {
                        } else if ((irqRegs & IRQ_SYNCWORD_ERROR) == IRQ_SYNCWORD_ERROR) {
                        } else {
                        	onRxDone();
                        }
                    }
                    if ((irqRegs & IRQ_SYNCWORD_VALID) == IRQ_SYNCWORD_VALID) {
                    	onSyncWordDone();
                    }
                    if ((irqRegs & IRQ_TX_DONE) == IRQ_TX_DONE) {
                    	onTxDone();
                    }
                    break;
                case MODE_TX:
                    if ((irqRegs & IRQ_TX_DONE) == IRQ_TX_DONE) {
                    	onTxDone();
                    }
                    break;
                default:
                    // Unexpected IRQ: silently returns
                    break;
            }
            break;
        case PACKET_TYPE_LORA:
            switch (operatingMode) {
                case MODE_RX:
                    if ((irqRegs & IRQ_RX_DONE) == IRQ_RX_DONE) {
                        if ((irqRegs & IRQ_CRC_ERROR) != IRQ_CRC_ERROR) {
                        	onRxDone();
                        }
                    }
                    break;
                case MODE_TX:
                    if ((irqRegs & IRQ_TX_DONE) == IRQ_TX_DONE) {
                    	onTxDone();
                    }
                    break;
                default:
                    // Unexpected IRQ: silently returns
                    break;
            }
            break;
        default:
            // Unexpected IRQ: silently returns
            break;
    }
}

//
// Private methods
//

inline void SX1280::waitOnBusy(void) {
	while (busyPin.isSet()) { }
}

void SX1280::readCommand(RadioCommands_t command, uint8_t *buffer, uint16_t size) {
    waitOnBusy( );

    if (radioSpi != NULL) {
    	ncsPin.low();
        if (command == RADIO_GET_STATUS) {
        	uint8_t txBuffer[3] = { (uint8_t)command, 0, 0 };
            uint8_t rxBuffer[sizeof(txBuffer)];

           	HAL_SPI_TransmitReceive(this->radioSpi, txBuffer, rxBuffer, sizeof(txBuffer), 1000);
           	buffer[0] = rxBuffer[2];
        } else {
        	const uint8_t preambleSize = 2;
        	uint8_t txBuffer[size + preambleSize] = { (uint8_t)command, 0 };
            uint8_t rxBuffer[sizeof(txBuffer)];

    		memset(txBuffer + preambleSize, 0, size);

        	HAL_SPI_TransmitReceive(this->radioSpi, txBuffer, rxBuffer, size + preambleSize, 1000);
        	memcpy(buffer, rxBuffer + preambleSize, size);
        }

    	ncsPin.high();
    }
}

void SX1280::writeCommand(RadioCommands_t command, uint8_t *buffer, uint16_t size) {
    waitOnBusy();

    if (radioSpi != NULL) {
    	ncsPin.low();

    	const uint8_t preambleSize = 1;
    	uint8_t txBuffer[size + preambleSize] = { (uint8_t)command };
		uint8_t rxBuffer[sizeof(txBuffer)];

		memcpy(txBuffer + preambleSize, buffer, size);

		HAL_SPI_TransmitReceive(this->radioSpi, txBuffer, rxBuffer, size + preambleSize, 1000);
    	ncsPin.high();
    }

    if (command != RADIO_SET_SLEEP) {
        waitOnBusy();
    }
}

void SX1280::readRegister(uint16_t address, uint8_t *buffer, uint16_t size) {
    waitOnBusy();

    if (radioSpi != NULL) {
    	ncsPin.low();

    	const uint8_t preambleSize = 4;
    	uint8_t txBuffer[size + preambleSize] = { RADIO_READ_REGISTER, (uint8_t)(address >> 8), (uint8_t)(address), 0 };
    	uint8_t rxBuffer[sizeof(txBuffer)];

		memset(txBuffer + preambleSize, 0, size);

    	HAL_SPI_TransmitReceive(this->radioSpi, txBuffer, rxBuffer, size + preambleSize, 1000);
    	memcpy(buffer, rxBuffer + preambleSize, size);
    	ncsPin.high();
    }
}

uint8_t SX1280::readRegister(uint16_t address) {
    uint8_t data;

    readRegister(address, &data, 1);
    return data;
}

void SX1280::writeRegister(uint16_t address, uint8_t *buffer, uint16_t size) {
    waitOnBusy();

    if (radioSpi != NULL) {
    	ncsPin.low();

    	const uint8_t preambleSize = 3;
    	uint8_t txBuffer[size + preambleSize] = { RADIO_WRITE_REGISTER, (uint8_t)(address >> 8), (uint8_t)(address) };
    	uint8_t rxBuffer[sizeof(txBuffer)];

		memcpy(txBuffer + preambleSize, buffer, size);

        HAL_SPI_TransmitReceive(this->radioSpi, txBuffer, rxBuffer, size + preambleSize, 1000);
    	ncsPin.high();
    }
}

void SX1280::writeRegister(uint16_t address, uint8_t value) {
    writeRegister(address, &value, 1);
}

void SX1280::readBuffer(uint8_t offset, uint8_t *buffer, uint8_t size) {
    waitOnBusy();

    if (radioSpi != NULL) {
    	ncsPin.low();

    	const uint8_t preambleSize = 3;
    	uint8_t txBuffer[size + preambleSize] = { RADIO_READ_BUFFER, offset, 0 };
		uint8_t rxBuffer[sizeof(txBuffer)];

		memset(txBuffer + preambleSize, 0, size);

		HAL_SPI_TransmitReceive(this->radioSpi, txBuffer, rxBuffer, size + preambleSize, 1000);
		memcpy(buffer, rxBuffer + preambleSize, size);
    	ncsPin.high();
    }
}

void SX1280::writeBuffer(uint8_t offset, uint8_t *buffer, uint8_t size) {
    waitOnBusy();

    if (radioSpi != NULL) {
    	ncsPin.low();

    	const uint8_t preambleSize = 2;
    	uint8_t txBuffer[size + preambleSize] = { RADIO_WRITE_BUFFER, offset };
		uint8_t rxBuffer[sizeof(txBuffer)];

		memcpy(txBuffer + preambleSize, buffer, size);

		HAL_SPI_TransmitReceive(this->radioSpi, txBuffer, rxBuffer, size + preambleSize, 1000);
    	ncsPin.high();
    }
}

void SX1280::resetChip(void) {
//    __disable_irq();

    HAL_Delay(20);
    nresetPin.low();
    HAL_Delay(50);
    nresetPin.high();
    HAL_Delay(20);
//    __enable_irq();
}

const std::array<uint32_t, 83> SX1280::frequencyLookUpTable {
	2400500000UL,
	2401500000UL,
	2402500000UL,
	2403500000UL,
	2404500000UL,
	2405500000UL,
	2406500000UL,
	2407500000UL,
	2408500000UL,
	2409500000UL,
	2410500000UL,
	2411500000UL,
	2412500000UL,
	2413500000UL,
	2414500000UL,
	2415500000UL,
	2416500000UL,
	2417500000UL,
	2418500000UL,
	2419500000UL,
	2420500000UL,
	2421500000UL,
	2422500000UL,
	2423500000UL,
	2424500000UL,
	2425500000UL,
	2426500000UL,
	2427500000UL,
	2428500000UL,
	2429500000UL,
	2430500000UL,
	2431500000UL,
	2432500000UL,
	2433500000UL,
	2434500000UL,
	2435500000UL,
	2436500000UL,
	2437500000UL,
	2438500000UL,
	2439500000UL,
	2440500000UL,
	2441500000UL,
	2442500000UL,
	2443500000UL,
	2444500000UL,
	2445500000UL,
	2446500000UL,
	2447500000UL,
	2448500000UL,
	2449500000UL,
	2450500000UL,
	2451500000UL,
	2452500000UL,
	2453500000UL,
	2454500000UL,
	2455500000UL,
	2456500000UL,
	2457500000UL,
	2458500000UL,
	2459500000UL,
	2460500000UL,
	2461500000UL,
	2462500000UL,
	2463500000UL,
	2464500000UL,
	2465500000UL,
	2466500000UL,
	2467500000UL,
	2468500000UL,
	2469500000UL,
	2470500000UL,
	2471500000UL,
	2472500000UL,
	2473500000UL,
	2474500000UL,
	2475500000UL,
	2476500000UL,
	2477500000UL,
	2478500000UL,
	2479500000UL,
	2480500000UL,
	2481500000UL,
	2482500000UL,
};

/**
 * @file altitudesensor.cpp
 * @brief Wrapper class for BMP388 pressure sensor.
 *
 * @author Varadi, Gyorgy, aka bodri
 * Contact: bodri@bodrico.com
 *
 * @bug No known bugs.
 *
 * MIT license, all text above must be included in any redistribution
 *
 */

#include "altitudesensor.h"
#include "main.h"
#include "i2c.h"

#include <cmath>
#include <cstring>
#include <optional>

static int8_t i2cWrite(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len);
static int8_t i2cRead(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len);
static void delayMilliSeconds(uint32_t ms);

AltitudeSensor::AltitudeSensor(uint8_t i2cAddress) :
	i2cAddress(i2cAddress << 1) {
	TelemetryData *sensor = new TelemetryData(0, "Vario Sensor", "", int8_tdt, 0);
	temperature = new TelemetryData(1, "Temperature", "°C", int8_tdt, 0);
	pressure = new TelemetryData(2, "Pressure", "°", int24_tdt, 2);

	this->telemetryDataArray = {
		  sensor,
		  temperature,
		  pressure,
	};

	// Calculate size
	for (auto &telemetryData : telemetryDataArray) {
		telemetryDataSize += telemetryData->valueSize() + 1;
	}

	sensorInfo.identifier = 0x2;
	sensorInfo.numberOfTelemetryData = telemetryDataArray.size() - 1;
	telemetryDataSize += sizeof(SensorInfo) - 1; // + sensorInfo - 1 for telemetryDataArray[0]
}

bool AltitudeSensor::start() {
	sensorDevice.dev_id = i2cAddress;
	sensorDevice.intf = BMP3_I2C_INTF;
	sensorDevice.read = &i2cRead;
	sensorDevice.write = &i2cWrite;

	sensorDevice.delay_ms = delayMilliSeconds;

	if (bmp3_init(&sensorDevice) != BMP3_OK) {
		return false;
	}

    sensorDevice.settings.press_en = BMP3_ENABLE;
    sensorDevice.settings.temp_en = BMP3_ENABLE;
    sensorDevice.settings.odr_filter.press_os = BMP3_OVERSAMPLING_8X;
    sensorDevice.settings.odr_filter.temp_os = BMP3_NO_OVERSAMPLING;
    sensorDevice.settings.odr_filter.odr = BMP3_ODR_50_HZ;
    sensorDevice.settings.odr_filter.iir_filter = BMP3_IIR_FILTER_COEFF_3;
    uint16_t settingsSelection = BMP3_PRESS_EN_SEL | BMP3_TEMP_EN_SEL | BMP3_PRESS_OS_SEL | BMP3_TEMP_OS_SEL | BMP3_ODR_SEL | BMP3_IIR_FILTER_SEL;
    if (bmp3_set_sensor_settings(settingsSelection, &sensorDevice) != BMP3_OK) {
    	return false;
    }

    // Set the power mode to normal mode
    sensorDevice.settings.op_mode = BMP3_NORMAL_MODE;
    if (bmp3_set_op_mode(&sensorDevice) != BMP3_OK) {
    	return false;
    }

    return true;
}

std::string AltitudeSensor::getDescription() {
	return "";
}

size_t AltitudeSensor::dataSize() {
	return telemetryDataSize;
}

std::string AltitudeSensor::getData() {
	std::string buffer;

	std::shared_ptr<bmp3_data> data = performReading();
	if (data) {
		temperature->setValue((int8_t)data->temperature);
		pressure->setValue((int32_t)data->pressure);

		buffer.resize(2);
		buffer[0] = (uint16_t)sensorInfo >> 8;
		buffer[1] = (uint16_t)sensorInfo;
		for (auto &telemetryData : telemetryDataArray) {
			buffer.append(telemetryData->getValueRepresentation());
		}
	}

	return buffer;
}

//double AltitudeSensor::readAltitude(double seaLevel) {
//	// Equation taken from BMP180 datasheet (page 16):
//	//  http://www.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf
//
//	// Note that using the equation from wikipedia can give bad results
//	// at high altitude. See this thread for more information:
//	//  http://forums.adafruit.com/viewtopic.php?f=22&t=58064
//
//	double atmospheric = readPressure() / 100.0F;
//	return 44330.0 * (1.0 - pow(atmospheric / seaLevel, 0.1903));
//}

std::shared_ptr<bmp3_data> AltitudeSensor::performReading(void) {
    struct bmp3_data data;

    if (bmp3_get_sensor_data(BMP3_ALL, &data, &sensorDevice) != BMP3_OK) {
    	return nullptr;
    }

	return std::make_shared<bmp3_data>(data);
}

int8_t i2cRead(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len) {
	if (HAL_I2C_Mem_Read(&hi2c3, dev_id, reg_addr, 1, reg_data, len, 1000) == HAL_OK) {
		Error_Handler();
	}

	return 0;
}

int8_t i2cWrite(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len) {
	if (HAL_I2C_Mem_Write(&hi2c3, dev_id, reg_addr, 1, reg_data, len, 1000) == HAL_OK) {
		Error_Handler();
	}

	return 0;
}

static void delayMilliSeconds(uint32_t ms) {
	HAL_Delay(ms);
}

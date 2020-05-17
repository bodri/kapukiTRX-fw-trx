/**
 * @file orientationsensor.cpp
 * @brief Wrapper class for BNO055 orientation sensor.
 *
 * @author Varadi, Gyorgy, aka bodri
 * Contact: bodri@bodrico.com
 *
 * @bug No known bugs.
 *
 * MIT license, all text above must be included in any redistribution
 *
 */

#include "orientationsensor.h"
#include "main.h"
#include "i2c.h"

#include <climits>
#include <cmath>
#include <memory>

static s8 i2cRead(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
static s8 i2cWrite(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
static void delayMilliSeconds(u32 msec);

OrientationSensor::OrientationSensor(uint8_t i2cAddress) :
	i2cAddress(i2cAddress << 1) {

	TelemetryData *sensor = new TelemetryData(0, "Orientation Sensor", "", int8_tdt, 0);
	temperature = new TelemetryData(1, "Temperature", "°C", int8_tdt, 0);
	yawAngle = new TelemetryData(2, "Yaw angle", "°", int16_tdt, 2);
	pitchAngle = new TelemetryData(3, "Pitch angle", "°", int16_tdt, 2);
	rollAngle = new TelemetryData(4, "Roll angle", "°", int16_tdt, 2);

	this->telemetryDataArray = {
		  sensor,
		  temperature,
		  yawAngle,
		  pitchAngle,
		  rollAngle
	};

	// Calculate size
	for (auto &telemetryData : telemetryDataArray) {
		telemetryDataSize += telemetryData->valueSize() + 1;
	}

	sensorInfo.identifier = 0x1;
	sensorInfo.numberOfTelemetryData = telemetryDataArray.size() - 1;
	telemetryDataSize += sizeof(SensorInfo) - 1; // + sensorInfo - 1 for telemetryDataArray[0]
}

bool OrientationSensor::start() {
	sensorDevice.dev_addr = i2cAddress;
	sensorDevice.bus_read = &i2cRead;
	sensorDevice.bus_write = &i2cWrite;
	sensorDevice.delay_msec = &delayMilliSeconds;

	if (bno055_init(&sensorDevice) != BNO055_SUCCESS) {
		return false;
	}

	BNO055_RETURN_FUNCTION_TYPE result = BNO055_SUCCESS;
	result += bno055_set_operation_mode(BNO055_OPERATION_MODE_CONFIG);

	//	Reset
	result += bno055_set_intr_rst(BNO055_BIT_ENABLE);
	HAL_Delay(100);
//	HAL_Delay(30);
//	while (read8(BNO055_CHIP_ID_ADDR) != BNO055_ID) {
//		HAL_Delay(10);
//	}
//	HAL_Delay(50);

	result += bno055_set_power_mode(BNO055_POWER_MODE_NORMAL);
	// Use external oscillator
	result += bno055_set_clk_src(BNO055_BIT_DISABLE);
	HAL_Delay(10);

	result += bno055_set_operation_mode(BNO055_OPERATION_MODE_NDOF);
	HAL_Delay(20);

	return result == BNO055_SUCCESS;
}

std::string OrientationSensor::getDescription() {
	return "";
}

size_t OrientationSensor::dataSize() {
	return telemetryDataSize;
}

std::string OrientationSensor::getData() {
	std::string buffer;

	s8 temp = getTemperature();
	bno055_euler_double_t eulerVector = getEulerVector();
	temperature->setValue(temp);
	yawAngle->setValue((int16_t)(eulerVector.h * 100));
	pitchAngle->setValue((int16_t)(eulerVector.r * 100));
	rollAngle->setValue((int16_t)(eulerVector.p * 100));


	buffer.resize(2);
	buffer[0] = (uint16_t)sensorInfo >> 8;
	buffer[1] = (uint16_t)sensorInfo;
	for (auto &telemetryData : telemetryDataArray) {
		buffer.append(telemetryData->getValueRepresentation());
	}

	return buffer;
}

int8_t OrientationSensor::getTemperature() {
	s8 temp;
	bno055_read_temp_data(&temp);
	return temp;
}

bno055_quaternion_t OrientationSensor::getQuaternion() {
	bno055_quaternion_t quaternion_wxyz;
	bno055_read_quaternion_wxyz(&quaternion_wxyz);
	return quaternion_wxyz;
}

bno055_euler_double_t OrientationSensor::getEulerVector() {
	struct bno055_euler_double_t d_euler_hpr;
	bno055_convert_double_euler_hpr_deg(&d_euler_hpr);
	return d_euler_hpr;
}

/*!
 *  @brief  Checks of all cal status values are set to 3 (fully calibrated)
 *  @return status of calibration
 */
//bool OrientationSensor::isFullyCalibrated() {
//	uint8_t system, gyro, accel, mag;
//	getCalibration(&system, &gyro, &accel, &mag);
//
//	switch (mode) {
//	case OPERATION_MODE_ACCONLY:
//		return (accel == 3);
//	case OPERATION_MODE_MAGONLY:
//		return (mag == 3);
//	case OPERATION_MODE_GYRONLY:
//	case OPERATION_MODE_M4G: /* No magnetometer calibration required. */
//		return (gyro == 3);
//	case OPERATION_MODE_ACCMAG:
//	case OPERATION_MODE_COMPASS:
//		return (accel == 3 && mag == 3);
//	case OPERATION_MODE_ACCGYRO:
//	case OPERATION_MODE_IMUPLUS:
//		return (accel == 3 && gyro == 3);
//	case OPERATION_MODE_MAGGYRO:
//		return (mag == 3 && gyro == 3);
//	default:
//		return (system == 3 && gyro == 3 && accel == 3 && mag == 3);
//	}
//}

void OrientationSensor::enterSuspendMode() {
	u8 modeBackup;
	bno055_get_operation_mode(&modeBackup);
	bno055_set_operation_mode(BNO055_OPERATION_MODE_CONFIG);
	HAL_Delay(25);

	bno055_set_power_mode(BNO055_POWER_MODE_SUSPEND);
	bno055_set_operation_mode(modeBackup);
	HAL_Delay(20);
}

void OrientationSensor::enterNormalMode() {
	u8 modeBackup;
	bno055_get_operation_mode(&modeBackup);
	bno055_set_operation_mode(BNO055_OPERATION_MODE_CONFIG);
	HAL_Delay(25);

	bno055_set_power_mode(BNO055_POWER_MODE_NORMAL);
	bno055_set_operation_mode(modeBackup);
	HAL_Delay(20);
}

static s8 i2cRead(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt) {
	if (HAL_I2C_Mem_Read(&hi2c3, dev_addr, reg_addr, 1, reg_data, cnt, 1000) == HAL_OK) {
		Error_Handler();
	}

	return 0;
}

static s8 i2cWrite(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt) {
	if (HAL_I2C_Mem_Write(&hi2c3, dev_addr, reg_addr, 1, reg_data, cnt, 1000) == HAL_OK) {
		Error_Handler();
	}

	return 0;
}

static void delayMilliSeconds(u32 msec) {
	HAL_Delay(msec);
}

/*!
 * @file Adafruit_BNO055.cpp
 *
 *  @mainpage Adafruit BNO055 Orientation Sensor
 *
 *  @section intro_sec Introduction
 *
 *  This is a library for the BNO055 orientation sensor
 *
 *  Designed specifically to work with the Adafruit BNO055 Breakout.
 *
 *  Pick one up today in the adafruit shop!
 *  ------> https://www.adafruit.com/product/2472
 *
 *  These sensors use I2C to communicate, 2 pins are required to interface.
 *
 *  Adafruit invests time and resources providing this open source code,
 *  please support Adafruit andopen-source hardware by purchasing products
 *  from Adafruit!
 *
 *  @section author Author
 *
 *  K.Townsend (Adafruit Industries)
 *
 *  @section license License
 *
 *  MIT license, all text above must be included in any redistribution
 */

#include "main.h"
#include "i2c.h"

#include <limits.h>
#include <math.h>
#include "orientationsensor.h"

// Our hardware interface functions
static s8 BNO055_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
static s8 BNO055_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
static void BNO055_delay_msek(u32 msek);

/*!
 *  @brief  Instantiates a new Adafruit_BNO055 class
 *  @param  sensorID
 *          sensor ID
 *  @param  address
 *          i2c address
 *  @param  theWire
 *          Wire object
 */
OrientationSensor::OrientationSensor(uint8_t i2cAddress) :
	i2cAddress(i2cAddress << 1) {
}

/*!
 *  @brief  Sets up the HW
 *  @param  mode
 *          mode values
 *           [OPERATION_MODE_CONFIG,
 *            OPERATION_MODE_ACCONLY,
 *            OPERATION_MODE_MAGONLY,
 *            OPERATION_MODE_GYRONLY,
 *            OPERATION_MODE_ACCMAG,
 *            OPERATION_MODE_ACCGYRO,
 *            OPERATION_MODE_MAGGYRO,
 *            OPERATION_MODE_AMG,
 *            OPERATION_MODE_IMUPLUS,
 *            OPERATION_MODE_COMPASS,
 *            OPERATION_MODE_M4G,
 *            OPERATION_MODE_NDOF_FMC_OFF,
 *            OPERATION_MODE_NDOF]
 *  @return true if process is successful
 */
bool OrientationSensor::begin() {
	sensorDevice.dev_addr = i2cAddress;
	sensorDevice.bus_read = &BNO055_I2C_bus_read;
	sensorDevice.bus_write = &BNO055_I2C_bus_write;
	sensorDevice.delay_msec = &BNO055_delay_msek;

	if (bno055_init(&sensorDevice) != BNO055_SUCCESS) {
		return false;
	}

	/*  For initializing the BNO sensor it is required to the operation mode
	 * of the sensor as NORMAL
	 * Normal mode can set from the register
	 * Page - page0
	 * register - 0x3E
	 * bit positions - 0 and 1*/
//	power_mode = ;

	/* set the power mode as NORMAL*/
	bno055_set_power_mode (BNO055_POWER_MODE_NORMAL);
	bno055_set_operation_mode (BNO055_OPERATION_MODE_NDOF);
	bno055_set_clk_src(BNO055_BIT_ENABLE);


//	/* Make sure we have the right device */
//	uint8_t id = read8(BNO055_CHIP_ID_ADDR);
//	if (id != BNO055_ID) {
//		HAL_Delay(1000); // hold on for boot
//		id = read8(BNO055_CHIP_ID_ADDR);
//		if (id != BNO055_ID) {
//			return false; // still not? ok bail
//		}
//	}
//
//	/* Switch to config mode (just in case since this is the default) */
//	setMode(OPERATION_MODE_CONFIG);
//
//	/* Reset */
//	write8(BNO055_SYS_TRIGGER_ADDR, 0x20);
//	/* Delay incrased to 30ms due to power issues https://tinyurl.com/y375z699 */
//	HAL_Delay(30);
//	while (read8(BNO055_CHIP_ID_ADDR) != BNO055_ID) {
//		HAL_Delay(10);
//	}
//	HAL_Delay(50);
//
//	/* Set to normal power mode */
//	write8(BNO055_PWR_MODE_ADDR, POWER_MODE_NORMAL);
//	HAL_Delay(10);
//
//	write8(BNO055_PAGE_ID_ADDR, 0);
//
//	/* Set the output units */
//	/*
//	 uint8_t unitsel = (0 << 7) | // Orientation = Android
//	 (0 << 4) | // Temperature = Celsius
//	 (0 << 2) | // Euler = Degrees
//	 (1 << 1) | // Gyro = Rads
//	 (0 << 0);  // Accelerometer = m/s^2
//	 write8(BNO055_UNIT_SEL_ADDR, unitsel);
//	 */
//
//	/* Configure axis mapping (see section 3.4) */
//	/*
//	 write8(BNO055_AXIS_MAP_CONFIG_ADDR, REMAP_CONFIG_P2); // P0-P7, Default is P1
//	 HAL_Delay(10);
//	 write8(BNO055_AXIS_MAP_SIGN_ADDR, REMAP_SIGN_P2); // P0-P7, Default is P1
//	 HAL_Delay(10);
//	 */
//
//	write8(BNO055_SYS_TRIGGER_ADDR, 0x0);
//	HAL_Delay(10);
//	/* Set the requested operating mode (see section 3.3) */
//	setMode(mode);
//	HAL_Delay(20);

	return true;
}

//
///*!
// *  @brief  Use the external 32.768KHz crystal
// *  @param  usextal
// *          use external crystal boolean
// */
//void OrientationSensor::setExtCrystalUse(bool usextal) {
//	bno055_opmode_t modeback = mode;
//
//	/* Switch to config mode (just in case since this is the default) */
//	setMode(OPERATION_MODE_CONFIG);
//	HAL_Delay(25);
//	write8(BNO055_PAGE_ID_ADDR, 0);
//	if (usextal) {
//		write8(BNO055_SYS_TRIGGER_ADDR, 0x80);
//	} else {
//		write8(BNO055_SYS_TRIGGER_ADDR, 0x00);
//	}
//	HAL_Delay(10);
//	/* Set the requested operating mode (see section 3.3) */
//	setMode(modeback);
//	HAL_Delay(20);
//}

/*!
 *  @brief  Gets the temperature in degrees celsius
 *  @return temperature in degrees celsius
 */
int8_t OrientationSensor::getTemperature() {
	s8 temp;
	bno055_read_temp_data(&temp);
	return temp;
}

/*!
 *  @brief  Gets a quaternion reading from the specified source
 *  @return quaternion reading
 */
bno055_quaternion_t OrientationSensor::getQuaternion() {
	bno055_quaternion_t quaternion_wxyz;
	bno055_read_quaternion_wxyz(&quaternion_wxyz);
	return quaternion_wxyz;
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

///*!
// *  @brief  Enter Suspend mode (i.e., sleep)
// */
//void OrientationSensor::enterSuspendMode() {
//	bno055_opmode_t modeback = mode;
//
//	/* Switch to config mode (just in case since this is the default) */
//	setMode(OPERATION_MODE_CONFIG);
//	HAL_Delay(25);
//	write8(BNO055_PWR_MODE_ADDR, 0x02);
//	/* Set the requested operating mode (see section 3.3) */
//	setMode(modeback);
//	HAL_Delay(20);
//}
//
///*!
// *  @brief  Enter Normal mode (i.e., wake)
// */
//void OrientationSensor::enterNormalMode() {
//	bno055_opmode_t modeback = mode;
//
//	/* Switch to config mode (just in case since this is the default) */
//	setMode(OPERATION_MODE_CONFIG);
//	HAL_Delay(25);
//	write8(BNO055_PWR_MODE_ADDR, 0x00);
//	/* Set the requested operating mode (see section 3.3) */
//	setMode(modeback);
//	HAL_Delay(20);
//}

static s8 BNO055_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt) {
	if (HAL_I2C_Mem_Read(&hi2c1, dev_addr, reg_addr, 1, reg_data, cnt, 1000) == HAL_OK) {
		Error_Handler();
	}

	return 0;
}

/**************************************************************************/
/*!
 @brief  Writes 8 bit values over I2C
 */
/**************************************************************************/
static s8 BNO055_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt) {
	if (HAL_I2C_Mem_Write(&hi2c1, dev_addr, reg_addr, 1, reg_data, cnt, 1000) == HAL_OK) {
		Error_Handler();
	}

	return 0;
}

static void BNO055_delay_msek(u32 msek) {
	HAL_Delay(msek);
}

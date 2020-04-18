/*!
 * @file Adafruit_BMP3XX.cpp
 *
 * @mainpage Adafruit BMP3XX temperature & barometric pressure sensor driver
 *
 * @section intro_sec Introduction
 *
 * This is the documentation for Adafruit's BMP3XX driver for the
 * Arduino platform.  It is designed specifically to work with the
 * Adafruit BMP388 breakout: https://www.adafruit.com/products/3966
 *
 * These sensors use I2C or SPI to communicate
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * @section author Author
 *
 * Written by Ladyada for Adafruit Industries.
 *
 * @section license License
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */

#include "bmp3xx.h"
#include "main.h"
#include "i2c.h"

#include <cmath>
#include <cstring>

// Our hardware interface functions
static int8_t i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len);
static int8_t i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len);
static void delay_msec(uint32_t ms);

/***************************************************************************
 PUBLIC FUNCTIONS
 ***************************************************************************/

/**************************************************************************/
/*!
 @brief  Instantiates sensor with Hardware SPI or I2C.
 @param  cspin SPI chip select. If not passed in, I2C will be used
 */
/**************************************************************************/
Bmp3xx::Bmp3xx() {
	filterEnabled = temperatureOversamplingEnabled = pressureOversamplingEnabled = false;
}

/**************************************************************************/
/*!
 @brief Initializes the sensor

 Hardware ss initialized, verifies it is in the I2C or SPI bus, then reads
 calibration data in preparation for sensor reads.

 @param  addr Optional parameter for the I2C address of BMP3. Default is 0x77
 @param  theWire Optional parameter for the I2C device we will use. Default is "Wire"
 @return True on sensor initialization success. False on failure.
 */
/**************************************************************************/
bool Bmp3xx::begin(uint8_t addr) {
	i2cAddress = addr;

//	_BMP3_i2c = theWire;
//	_BMP3_i2c->begin();

	sensorDevice.dev_id = addr;
	sensorDevice.intf = BMP3_I2C_INTF;
	sensorDevice.read = &i2c_read;
	sensorDevice.write = &i2c_write;

	sensorDevice.delay_ms = delay_msec;

	int8_t rslt = BMP3_OK;
	rslt = bmp3_init(&sensorDevice);

	if (rslt != BMP3_OK)
		return false;

#ifdef BMP3XX_DEBUG
  Serial.print("T1 = "); Serial.println(sensorDevice.calib_data.reg_calib_data.par_t1);
  Serial.print("T2 = "); Serial.println(sensorDevice.calib_data.reg_calib_data.par_t2);
  Serial.print("T3 = "); Serial.println(sensorDevice.calib_data.reg_calib_data.par_t3);
  Serial.print("P1 = "); Serial.println(sensorDevice.calib_data.reg_calib_data.par_p1);
  Serial.print("P2 = "); Serial.println(sensorDevice.calib_data.reg_calib_data.par_p2);
  Serial.print("P3 = "); Serial.println(sensorDevice.calib_data.reg_calib_data.par_p3);
  Serial.print("P4 = "); Serial.println(sensorDevice.calib_data.reg_calib_data.par_p4);
  Serial.print("P5 = "); Serial.println(sensorDevice.calib_data.reg_calib_data.par_p5);
  Serial.print("P6 = "); Serial.println(sensorDevice.calib_data.reg_calib_data.par_p6);
  Serial.print("P7 = "); Serial.println(sensorDevice.calib_data.reg_calib_data.par_p7);
  Serial.print("P8 = "); Serial.println(sensorDevice.calib_data.reg_calib_data.par_p8);
  Serial.print("P9 = "); Serial.println(sensorDevice.calib_data.reg_calib_data.par_p9);
  Serial.print("P10 = "); Serial.println(sensorDevice.calib_data.reg_calib_data.par_p10);
  Serial.print("P11 = "); Serial.println(sensorDevice.calib_data.reg_calib_data.par_p11);
  //Serial.print("T lin = "); Serial.println(the_sensor.calib_data.reg_calib_data.t_lin);
#endif

	setTemperatureOversampling(BMP3_NO_OVERSAMPLING);
	setPressureOversampling(BMP3_NO_OVERSAMPLING);
	setIIRFilterCoeff(BMP3_IIR_FILTER_DISABLE);

	// don't do anything till we request a reading
	sensorDevice.settings.op_mode = BMP3_FORCED_MODE;

	return true;
}

/**************************************************************************/
/*!
 @brief Performs a reading and returns the ambient temperature.
 @return Temperature in degrees Centigrade
 */
/**************************************************************************/
float Bmp3xx::readTemperature(void) {
	performReading();
	return temperature;
}

/**************************************************************************/
/*!
 @brief Performs a reading and returns the barometric pressure.
 @return Barometic pressure in Pascals
 */
/**************************************************************************/
float Bmp3xx::readPressure(void) {
	performReading();
	return pressure;
}

/**************************************************************************/
/*!
 @brief Calculates the altitude (in meters).

 Reads the current atmostpheric pressure (in hPa) from the sensor and calculates
 via the provided sea-level pressure (in hPa).

 @param  seaLevel      Sea-level pressure in hPa
 @return Altitude in meters
 */
/**************************************************************************/
float Bmp3xx::readAltitude(float seaLevel) {
	// Equation taken from BMP180 datasheet (page 16):
	//  http://www.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf

	// Note that using the equation from wikipedia can give bad results
	// at high altitude. See this thread for more information:
	//  http://forums.adafruit.com/viewtopic.php?f=22&t=58064

	float atmospheric = readPressure() / 100.0F;
	return 44330.0 * (1.0 - pow(atmospheric / seaLevel, 0.1903));
}

/**************************************************************************/
/*!
 @brief Performs a full reading of all sensors in the BMP3XX.

 Assigns the internal Adafruit_BMP3XX#temperature & Adafruit_BMP3XX#pressure member variables

 @return True on success, False on failure
 */
/**************************************************************************/
bool Bmp3xx::performReading(void) {
	int8_t rslt;
	/* Used to select the settings user needs to change */
	uint16_t settings_sel = 0;
	/* Variable used to select the sensor component */
	uint8_t sensor_comp = 0;

	/* Select the pressure and temperature sensor to be enabled */
	sensorDevice.settings.temp_en = BMP3_ENABLE;
	settings_sel |= BMP3_TEMP_EN_SEL;
	sensor_comp |= BMP3_TEMP;
	if (temperatureOversamplingEnabled) {
		settings_sel |= BMP3_TEMP_OS_SEL;
	}

	sensorDevice.settings.press_en = BMP3_ENABLE;
	settings_sel |= BMP3_PRESS_EN_SEL;
	sensor_comp |= BMP3_PRESS;
	if (pressureOversamplingEnabled) {
		settings_sel |= BMP3_PRESS_OS_SEL;
	}

	if (filterEnabled) {
		settings_sel |= BMP3_IIR_FILTER_SEL;
	}

	if (outputDataRateEnabled) {
		settings_sel |= BMP3_ODR_SEL;
	}

	// set interrupt to data ready
	//settings_sel |= BMP3_DRDY_EN_SEL | BMP3_LEVEL_SEL | BMP3_LATCH_SEL;

	/* Set the desired sensor configuration */
	rslt = bmp3_set_sensor_settings(settings_sel, &sensorDevice);
	if (rslt != BMP3_OK)
		return false;

	/* Set the power mode */
	sensorDevice.settings.op_mode = BMP3_FORCED_MODE;
	rslt = bmp3_set_op_mode(&sensorDevice);
	if (rslt != BMP3_OK)
		return false;

	/* Variable used to store the compensated data */
	struct bmp3_data data;

	/* Temperature and Pressure data are read and stored in the bmp3_data instance */
	rslt = bmp3_get_sensor_data(sensor_comp, &data, &sensorDevice);

	/* Save the temperature and pressure data */
	temperature = data.temperature;
	pressure = data.pressure;
	if (rslt != BMP3_OK)
		return false;

	return true;
}

/**************************************************************************/
/*!
 @brief  Setter for Temperature oversampling
 @param  oversample Oversampling setting, can be BMP3_NO_OVERSAMPLING, BMP3_OVERSAMPLING_2X, BMP3_OVERSAMPLING_4X, BMP3_OVERSAMPLING_8X, BMP3_OVERSAMPLING_16X, BMP3_OVERSAMPLING_32X
 @return True on success, False on failure
 */
/**************************************************************************/

bool Bmp3xx::setTemperatureOversampling(uint8_t oversample) {
	if (oversample > BMP3_OVERSAMPLING_32X)
		return false;

	sensorDevice.settings.odr_filter.temp_os = oversample;

	if (oversample == BMP3_NO_OVERSAMPLING)
		temperatureOversamplingEnabled = false;
	else
		temperatureOversamplingEnabled = true;

	return true;
}

/**************************************************************************/
/*!
 @brief  Setter for Pressure oversampling
 @param  oversample Oversampling setting, can be BMP3_NO_OVERSAMPLING, BMP3_OVERSAMPLING_2X, BMP3_OVERSAMPLING_4X, BMP3_OVERSAMPLING_8X, BMP3_OVERSAMPLING_16X, BMP3_OVERSAMPLING_32X
 @return True on success, False on failure
 */
/**************************************************************************/
bool Bmp3xx::setPressureOversampling(uint8_t oversample) {
	if (oversample > BMP3_OVERSAMPLING_32X)
		return false;

	sensorDevice.settings.odr_filter.press_os = oversample;

	if (oversample == BMP3_NO_OVERSAMPLING)
		pressureOversamplingEnabled = false;
	else
		pressureOversamplingEnabled = true;

	return true;
}

/**************************************************************************/
/*!
 @brief  Setter for IIR filter coefficient
 @param filtercoeff Coefficient of the filter (in samples). Can be BMP3_IIR_FILTER_DISABLE (no filtering), BMP3_IIR_FILTER_COEFF_1, BMP3_IIR_FILTER_COEFF_3, BMP3_IIR_FILTER_COEFF_7, BMP3_IIR_FILTER_COEFF_15, BMP3_IIR_FILTER_COEFF_31, BMP3_IIR_FILTER_COEFF_63, BMP3_IIR_FILTER_COEFF_127
 @return True on success, False on failure

 */
/**************************************************************************/
bool Bmp3xx::setIIRFilterCoeff(uint8_t filtercoeff) {
	if (filtercoeff > BMP3_IIR_FILTER_COEFF_127)
		return false;

	sensorDevice.settings.odr_filter.iir_filter = filtercoeff;

	if (filtercoeff == BMP3_IIR_FILTER_DISABLE)
		filterEnabled = false;
	else
		filterEnabled = true;

	return true;
}

/**************************************************************************/
/*!
 @brief  Setter for output data rate (ODR)
 @param odr Sample rate in Hz. Can be BMP3_ODR_200_HZ, BMP3_ODR_100_HZ, BMP3_ODR_50_HZ, BMP3_ODR_25_HZ, BMP3_ODR_12_5_HZ, BMP3_ODR_6_25_HZ, BMP3_ODR_3_1_HZ, BMP3_ODR_1_5_HZ, BMP3_ODR_0_78_HZ, BMP3_ODR_0_39_HZ, BMP3_ODR_0_2_HZ, BMP3_ODR_0_1_HZ, BMP3_ODR_0_05_HZ, BMP3_ODR_0_02_HZ, BMP3_ODR_0_01_HZ, BMP3_ODR_0_006_HZ, BMP3_ODR_0_003_HZ, or BMP3_ODR_0_001_HZ 
 @return True on success, False on failure

 */
/**************************************************************************/
bool Bmp3xx::setOutputDataRate(uint8_t odr) {
	if (odr > BMP3_ODR_0_001_HZ)
		return false;

	sensorDevice.settings.odr_filter.odr = odr;

	outputDataRateEnabled = true;

	return true;
}

/**************************************************************************/
/*!
 @brief  Reads 8 bit values over I2C
 */
/**************************************************************************/
int8_t i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len) {

	if (HAL_I2C_Mem_Read(&hi2c1, dev_id, reg_addr, 1, reg_data, len, 1000) == HAL_OK) {
	}

	return 0;
}

/**************************************************************************/
/*!
 @brief  Writes 8 bit values over I2C
 */
/**************************************************************************/
int8_t i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len) {

//	uint8_t buffer[len + 1];
//	buffer[0] = reg_addr;
//	memcpy(buffer + 1, reg_data, len);
//	if (HAL_I2C_Master_Transmit(&hi2c1, dev_id, buffer, sizeof(buffer), 1000000) == HAL_OK) {
//	}

	if (HAL_I2C_Mem_Write(&hi2c1, dev_id, reg_addr, 1, reg_data, len, 1000) == HAL_OK) {
	}

	return 0;
}

static void delay_msec(uint32_t ms) {
	HAL_Delay(ms);
}

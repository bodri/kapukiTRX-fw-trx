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

#include <altitude/bmp388.h>
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
BMP388::BMP388(uint8_t i2sAddress) :
	i2cAddress(i2sAddress << 1) {
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
bool BMP388::begin() {
	sensorDevice.dev_id = i2cAddress;
	sensorDevice.intf = BMP3_I2C_INTF;
	sensorDevice.read = &i2c_read;
	sensorDevice.write = &i2c_write;

	sensorDevice.delay_ms = delay_msec;

	if (bmp3_init(&sensorDevice) != BMP3_OK) {
		return false;
	}

//	setTemperatureOversampling(BMP3_NO_OVERSAMPLING);
//	setPressureOversampling(BMP3_NO_OVERSAMPLING);
//	setIIRFilterCoeff(BMP3_IIR_FILTER_DISABLE);
//
//	// don't do anything till we request a reading
//	sensorDevice.settings.op_mode = BMP3_FORCED_MODE;

	return true;
}

int8_t BMP388::setNormalMode() {
    int8_t rslt;
    /* Used to select the settings user needs to change */
    uint16_t settings_sel;

    /* Select the pressure and temperature sensor to be enabled */
    sensorDevice.settings.press_en = BMP3_ENABLE;
    sensorDevice.settings.temp_en = BMP3_ENABLE;
    /* Select the output data rate and oversampling settings for pressure and temperature */
    sensorDevice.settings.odr_filter.press_os = BMP3_OVERSAMPLING_8X;
    sensorDevice.settings.odr_filter.temp_os = BMP3_NO_OVERSAMPLING;
    sensorDevice.settings.odr_filter.odr = BMP3_ODR_50_HZ;
    sensorDevice.settings.odr_filter.iir_filter = BMP3_IIR_FILTER_COEFF_3;
    /* Assign the settings which needs to be set in the sensor */
    settings_sel = BMP3_PRESS_EN_SEL | BMP3_TEMP_EN_SEL | BMP3_PRESS_OS_SEL | BMP3_TEMP_OS_SEL | BMP3_ODR_SEL | BMP3_IIR_FILTER_SEL;
    rslt = bmp3_set_sensor_settings(settings_sel, &sensorDevice);

    /* Set the power mode to normal mode */
    sensorDevice.settings.op_mode = BMP3_NORMAL_MODE;
    rslt = bmp3_set_op_mode(&sensorDevice);

    return rslt;
}

int8_t BMP388::setForcedMode() {
    int8_t rslt;
    /* Used to select the settings user needs to change */
    uint16_t settings_sel;

    /* Select the pressure and temperature sensor to be enabled */
    sensorDevice.settings.press_en = BMP3_ENABLE;
    sensorDevice.settings.temp_en = BMP3_ENABLE;
    /* Assign the settings which needs to be set in the sensor */
    settings_sel = BMP3_PRESS_EN_SEL | BMP3_TEMP_EN_SEL;
    /* Write the settings in the sensor */
    rslt = bmp3_set_sensor_settings(settings_sel, &sensorDevice);

    /* Select the power mode */
    sensorDevice.settings.op_mode = BMP3_FORCED_MODE;
    /* Set the power mode in the sensor */
    rslt = bmp3_set_op_mode(&sensorDevice);

    return rslt;
}

/**************************************************************************/
/*!
 @brief Performs a reading and returns the ambient temperature.
 @return Temperature in degrees Centigrade
 */
/**************************************************************************/
double BMP388::readTemperature(void) {
	performReading();
	return temperature;
}

/**************************************************************************/
/*!
 @brief Performs a reading and returns the barometric pressure.
 @return Barometic pressure in Pascals
 */
/**************************************************************************/
double BMP388::readPressure(void) {
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
double BMP388::readAltitude(double seaLevel) {
	// Equation taken from BMP180 datasheet (page 16):
	//  http://www.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf

	// Note that using the equation from wikipedia can give bad results
	// at high altitude. See this thread for more information:
	//  http://forums.adafruit.com/viewtopic.php?f=22&t=58064

	double atmospheric = readPressure() / 100.0F;
	return 44330.0 * (1.0 - pow(atmospheric / seaLevel, 0.1903));
}

/**************************************************************************/
/*!
 @brief Performs a full reading of all sensors in the BMP3XX.

 Assigns the internal Adafruit_BMP3XX#temperature & Adafruit_BMP3XX#pressure member variables

 @return True on success, False on failure
 */
/**************************************************************************/
bool BMP388::performReading(void) {
//	int8_t rslt;
//	/* Used to select the settings user needs to change */
//	uint16_t settings_sel = 0;
//	/* Variable used to select the sensor component */
//	uint8_t sensor_comp = 0;
//
//	/* Select the pressure and temperature sensor to be enabled */
//	sensorDevice.settings.temp_en = BMP3_ENABLE;
//	settings_sel |= BMP3_TEMP_EN_SEL;
//	sensor_comp |= BMP3_TEMP;
//	if (temperatureOversamplingEnabled) {
//		settings_sel |= BMP3_TEMP_OS_SEL;
//	}
//
//	sensorDevice.settings.press_en = BMP3_ENABLE;
//	settings_sel |= BMP3_PRESS_EN_SEL;
//	sensor_comp |= BMP3_PRESS;
//	if (pressureOversamplingEnabled) {
//		settings_sel |= BMP3_PRESS_OS_SEL;
//	}
//
//	if (filterEnabled) {
//		settings_sel |= BMP3_IIR_FILTER_SEL;
//	}
//
//	if (outputDataRateEnabled) {
//		settings_sel |= BMP3_ODR_SEL;
//	}
//
//	// set interrupt to data ready
//	//settings_sel |= BMP3_DRDY_EN_SEL | BMP3_LEVEL_SEL | BMP3_LATCH_SEL;
//
//	/* Set the desired sensor configuration */
//	rslt = bmp3_set_sensor_settings(settings_sel, &sensorDevice);
//	if (rslt != BMP3_OK)
//		return false;
//
//	/* Set the power mode */
//	sensorDevice.settings.op_mode = BMP3_FORCED_MODE;
//	rslt = bmp3_set_op_mode(&sensorDevice);
//	if (rslt != BMP3_OK)
//		return false;
//
//	/* Variable used to store the compensated data */
//	struct bmp3_data data;
//
//	/* Temperature and Pressure data are read and stored in the bmp3_data instance */
//	rslt = bmp3_get_sensor_data(sensor_comp, &data, &sensorDevice);

    int8_t rslt;
    /* Variable used to select the sensor component */
    uint8_t sensor_comp;
    /* Variable used to store the compensated data */
    struct bmp3_data data;

    /* Sensor component selection */
    sensor_comp = BMP3_PRESS | BMP3_TEMP;
    /* Temperature and Pressure data are read and stored in the bmp3_data instance */
    rslt = bmp3_get_sensor_data(sensor_comp, &data, &sensorDevice);

//    /* Print the temperature and pressure data */
//    printf("Temperature in deg celsius\t Pressure in Pascal\t\n");
//	#ifdef BMP3_DOUBLE_PRECISION_COMPENSATION
//    printf("%0.2f\t\t %0.2f\t\t\n",data.temperature, data.pressure);
//	#else
//	/* for fixed point the compensated temperature and pressure output has a multiplication factor of 100 */
//    printf("%lld\t\t %llu\t\t\n",data.temperature, data.pressure);
//	#endif

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

bool BMP388::setTemperatureOversampling(uint8_t oversample) {
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
bool BMP388::setPressureOversampling(uint8_t oversample) {
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
bool BMP388::setIIRFilterCoeff(uint8_t filtercoeff) {
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
bool BMP388::setOutputDataRate(uint8_t odr) {
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
		Error_Handler();
	}

	return 0;
}

/**************************************************************************/
/*!
 @brief  Writes 8 bit values over I2C
 */
/**************************************************************************/
int8_t i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len) {
	if (HAL_I2C_Mem_Write(&hi2c1, dev_id, reg_addr, 1, reg_data, len, 1000) == HAL_OK) {
		Error_Handler();
	}

	return 0;
}

static void delay_msec(uint32_t ms) {
	HAL_Delay(ms);
}

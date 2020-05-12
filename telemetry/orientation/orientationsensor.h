/**
 * @file orientationsensor.h
 * @brief Wrapper class for BNO055 orientation sensor.
 *
 * Sensor data:
 * 			Name			type		decimal point
 * 			------------------------------------------
 * 			"Temperature"	int8_t		0
 * 			"Yaw angle"		int16_t		2
 * 			"Pitch angle"	int16_t		2
 * 			"Roll angle"	int16_t		2
 *
 * @author Varadi, Gyorgy, aka bodri
 * Contact: bodri@bodrico.com
 *
 * @bug No known bugs.
 *
 * MIT license, all text above must be included in any redistribution
 *
 */

#ifndef __ORIENTATIONSENSOR_H__
#define __ORIENTATIONSENSOR_H__


#include "bno055.h"
#include "sensor.h"

#include <stdint.h>
#include <vector>

class OrientationSensor : public Sensor {
public:
	OrientationSensor(uint8_t i2cAddress = BNO055_I2C_ADDR1);

	virtual bool start() override;
	virtual size_t dataSize() override;
	virtual std::string getDescription() override;
	virtual std::string getData() override;

private:
	uint8_t i2cAddress;
	struct bno055_t sensorDevice;

	TelemetryData *temperature;
	TelemetryData *yawAngle;
	TelemetryData *pitchAngle;
	TelemetryData *rollAngle;
	std::vector<TelemetryData *> telemetryDataArray;
	size_t telemetryDataSize { 0 };

	int8_t getTemperature();
	bno055_quaternion_t getQuaternion();
	bno055_euler_double_t getEulerVector();

	void enterSuspendMode();
	void enterNormalMode();
};

#endif //__ORIENTATIONSENSOR_H__

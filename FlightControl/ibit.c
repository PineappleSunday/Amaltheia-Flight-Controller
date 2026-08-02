#include "ibit.h"

uint8_t IBIT_Evaluate(const LSM303* imu, const I3GD20* gyro, const IBIT_StatusInputs* status)
{
	uint8_t bits = 0u;

	if ((status != 0) && status->gps_ready) {
		bits |= IBIT_SENSOR_GPS;
	}

	if ((imu != 0) && (imu->variant != LSM303_UNKNOWN) && (imu->hi2c != 0)) {
		bits |= IBIT_SENSOR_ACCEL_MAG;
	}

	if ((status != 0) && status->bme280_ready) {
		bits |= IBIT_SENSOR_BME280;
	}

	if ((gyro != 0) && gyro->initialized) {
		bits |= IBIT_SENSOR_GYRO;
	}

	if ((status != 0) && status->lidar_ready) {
		bits |= IBIT_SENSOR_LIDAR;
	}

	if ((status != 0) && status->telemetry_ready) {
		bits |= IBIT_SENSOR_TELEMETRY;
	}

	return bits;
}

bool IBIT_HasRequired(uint8_t mask, uint8_t required_mask)
{
	return (mask & required_mask) == required_mask;
}

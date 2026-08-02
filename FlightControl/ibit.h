#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "i3gd20.h"
#include "lsm303.h"

typedef enum {
	IBIT_SENSOR_GPS        = 0x01u,
	IBIT_SENSOR_ACCEL_MAG  = 0x02u,
	IBIT_SENSOR_BME280     = 0x04u,
	IBIT_SENSOR_GYRO       = 0x08u,
	IBIT_SENSOR_LIDAR      = 0x10u,
	IBIT_SENSOR_TELEMETRY  = 0x20u,
	IBIT_SENSOR_SPARE_1    = 0x40u,
	IBIT_SENSOR_SPARE_2    = 0x80u
} IBIT_SensorMask;

typedef struct {
	bool gps_ready;
	bool bme280_ready;
	bool lidar_ready;
	bool telemetry_ready;
} IBIT_StatusInputs;

uint8_t IBIT_Evaluate(const LSM303* imu, const I3GD20* gyro, const IBIT_StatusInputs* status);
bool IBIT_HasRequired(uint8_t mask, uint8_t required_mask);

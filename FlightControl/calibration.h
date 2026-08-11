#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <stdbool.h>

typedef struct {
	float bias[3];
	float counts_per_g[3];
} AccelCalibration;

typedef struct {
	float gx_bias_dps;
	float gy_bias_dps;
	float gz_bias_dps;
	bool valid;
} GyroCalibration;

extern const AccelCalibration g_lsm303agr_accel_cal;
extern GyroCalibration g_gyro_calibration;

#endif // CALIBRATION_H

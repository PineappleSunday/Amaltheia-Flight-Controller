#include "calibration.h"

const AccelCalibration g_lsm303agr_accel_cal = {
	.bias = {
		25.0f,
		-19.0f,
		-40.0f
	},
	.counts_per_g = {
		1002.0f,
		1044.0f,
		1016.0f
	}
};

GyroCalibration g_gyro_calibration = {
	.gx_bias_dps = 0.0f,
	.gy_bias_dps = 0.0f,
	.gz_bias_dps = 0.0f,
	.valid = false
};

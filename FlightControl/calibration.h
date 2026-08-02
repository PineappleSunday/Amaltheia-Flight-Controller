#ifndef CALIBRATION_H
#define CALIBRATION_H

typedef struct {
	float bias[3];
	float counts_per_g[3];
} AccelCalibration;

extern const AccelCalibration g_lsm303agr_accel_cal;

#endif // CALIBRATION_H

#include "altitudeEst.h"

void AltitudeEst_Init(AltitudeEstimator* est, uint32_t now_ms)
{
	if (est == 0) {
		return;
	}

	est->initialized = true;
	est->bias_valid = false;
	est->boot_cal_complete = false;
	est->boot_sample_count = 0u;
	est->boot_start_ms = now_ms;
	est->last_update_ms = now_ms;
	est->gps_altitude_sum_m = 0.0f;
	est->bme_altitude_sum_m = 0.0f;
	est->bme_altitude_bias_m = 0.0f;
	est->corrected_bme_altitude_m = 0.0f;
}

void AltitudeEst_ResetBootCalibration(AltitudeEstimator* est, uint32_t now_ms)
{
	if (est == 0) {
		return;
	}

	est->bias_valid = false;
	est->boot_cal_complete = false;
	est->boot_sample_count = 0u;
	est->boot_start_ms = now_ms;
	est->last_update_ms = now_ms;
	est->gps_altitude_sum_m = 0.0f;
	est->bme_altitude_sum_m = 0.0f;
	est->bme_altitude_bias_m = 0.0f;
	est->corrected_bme_altitude_m = 0.0f;
}

static void complete_boot_calibration(AltitudeEstimator* est)
{
	if (est->boot_sample_count == 0u) {
		return;
	}

	const float inv_count = 1.0f / (float)est->boot_sample_count;
	const float gps_mean_m = est->gps_altitude_sum_m * inv_count;
	const float bme_mean_m = est->bme_altitude_sum_m * inv_count;

	est->bme_altitude_bias_m = gps_mean_m - bme_mean_m;
	est->corrected_bme_altitude_m = bme_mean_m + est->bme_altitude_bias_m;
	est->bias_valid = true;
	est->boot_cal_complete = true;
}

void AltitudeEst_Update(
		AltitudeEstimator* est,
		bool gps_valid,
		float gps_altitude_m,
		bool bme_valid,
		float bme_altitude_m,
		uint32_t now_ms)
{
	if (est == 0) {
		return;
	}

	if (!est->initialized) {
		AltitudeEst_Init(est, now_ms);
	}

	est->last_update_ms = now_ms;

	if (bme_valid) {
		est->corrected_bme_altitude_m = bme_altitude_m + est->bme_altitude_bias_m;
	}

	if (est->boot_cal_complete) {
		return;
	}

	if (gps_valid && bme_valid) {
		est->gps_altitude_sum_m += gps_altitude_m;
		est->bme_altitude_sum_m += bme_altitude_m;
		if (est->boot_sample_count < UINT16_MAX) {
			est->boot_sample_count++;
		}
	}

	if (est->boot_sample_count >= ALTITUDE_EST_BOOT_SAMPLE_TARGET) {
		complete_boot_calibration(est);
		return;
	}

	if ((now_ms - est->boot_start_ms) >= ALTITUDE_EST_BOOT_TIMEOUT_MS) {
		complete_boot_calibration(est);
	}
}

bool AltitudeEst_BiasValid(const AltitudeEstimator* est)
{
	return (est != 0) && est->bias_valid;
}

bool AltitudeEst_BootCalibrationComplete(const AltitudeEstimator* est)
{
	return (est != 0) && est->boot_cal_complete;
}

float AltitudeEst_GetBmeBiasM(const AltitudeEstimator* est)
{
	return (est != 0) ? est->bme_altitude_bias_m : 0.0f;
}

float AltitudeEst_GetCorrectedBmeAltitudeM(const AltitudeEstimator* est)
{
	return (est != 0) ? est->corrected_bme_altitude_m : 0.0f;
}

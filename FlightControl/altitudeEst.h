#ifndef ALTITUDE_EST_H
#define ALTITUDE_EST_H

#include <stdbool.h>
#include <stdint.h>

#define ALTITUDE_EST_BOOT_SAMPLE_TARGET 20u
#define ALTITUDE_EST_BOOT_TIMEOUT_MS    30000u

typedef struct {
	bool initialized;
	bool bias_valid;
	bool boot_cal_complete;
	uint16_t boot_sample_count;
	uint32_t boot_start_ms;
	uint32_t last_update_ms;
	float gps_altitude_sum_m;
	float bme_altitude_sum_m;
	float bme_altitude_bias_m;
	float corrected_bme_altitude_m;
} AltitudeEstimator;

void AltitudeEst_Init(AltitudeEstimator* est, uint32_t now_ms);
void AltitudeEst_ResetBootCalibration(AltitudeEstimator* est, uint32_t now_ms);
void AltitudeEst_Update(
		AltitudeEstimator* est,
		bool gps_valid,
		float gps_altitude_m,
		bool bme_valid,
		float bme_altitude_m,
		uint32_t now_ms);

bool AltitudeEst_BiasValid(const AltitudeEstimator* est);
bool AltitudeEst_BootCalibrationComplete(const AltitudeEstimator* est);
float AltitudeEst_GetBmeBiasM(const AltitudeEstimator* est);
float AltitudeEst_GetCorrectedBmeAltitudeM(const AltitudeEstimator* est);

#endif // ALTITUDE_EST_H

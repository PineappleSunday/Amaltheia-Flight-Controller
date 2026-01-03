// AHRS.c

#include "AHRS.h"
#include "kalman.h"
#include <math.h>

// Global Kalman instances defined in main.c
extern Kalman_t kf_roll;
extern Kalman_t kf_pitch;
extern Kalman_t kf_yaw;

static float wrap_deg(float a) {
	while (a > 180.0f) a -= 360.0f;
	while (a < -180.0f) a += 360.0f;
	return a;
}

/**
 * @brief Performs sensor fusion to update the global VehicleState.
 * Replicates logic from main.c 100Hz loop.
 */
void AHRS_Update(ahrsSensor_t* raw, vehicleState_t* state, float dt)
{
	/* -------------------------------------------------
	 * 1. Gyro axis mapping (matches working main.c)
	 * ------------------------------------------------- */
	float gx =  raw->gy;
	float gy =  raw->gx;
	float gz = -raw->gz;

	Kalman_Predict(&kf_roll,  gx, dt);
	Kalman_Predict(&kf_pitch, gy, dt);
	Kalman_Predict(&kf_yaw,   gz, dt);

	/* -------------------------------------------------
	 * 2. Accelerometer observation (DO NOT TOUCH)
	 * ------------------------------------------------- */
	float accel_roll =
	    atan2f(raw->ay, raw->az) * 57.29578f;

	float accel_pitch =
	    atan2f(-raw->ax,
	           sqrtf(raw->ay*raw->ay + raw->az*raw->az)) * 57.29578f;

	/* -------------------------------------------------
	 * 3. Magnetometer (tilt compensated yaw)
	 * ------------------------------------------------- */
	float phi   = kf_roll.angle  * 0.0174533f;
	float theta = kf_pitch.angle * 0.0174533f;

	float mx = raw->mx - 0.24f;
	float my = -(raw->my - 0.24f);
	float mz = -(raw->mz + 0.08f);

	float By = my * cosf(phi) - mz * sinf(phi);
	float Bx = mx * cosf(theta) +
	           (my * sinf(phi) + mz * cosf(phi)) * sinf(theta);

	float mag_yaw = atan2f(-By, Bx) * 57.29578f;

	while (mag_yaw > 180.0f) mag_yaw -= 360.0f;
	while (mag_yaw < -180.0f) mag_yaw += 360.0f;

	/* -------------------------------------------------
	 * 4. Kalman updates
	 * ------------------------------------------------- */
	state->roll  = Kalman_Update(&kf_roll,  accel_roll);
	state->pitch = Kalman_Update(&kf_pitch, accel_pitch);
	state->yaw   = Kalman_Update(&kf_yaw,   mag_yaw);

	/* -------------------------------------------------
	 * 5. Body rates (rad/s)
	 * ------------------------------------------------- */
	state->p = gx * 0.0174533f;
	state->q = gy * 0.0174533f;
	state->r = gz * 0.0174533f;
}

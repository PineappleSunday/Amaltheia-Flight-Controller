// AHRS.c

#include "AHRS.h"
#include "kalman.h"
#include <stdbool.h>
#include <math.h>

// Global Kalman instances defined in main.c
extern Kalman_t kf_roll;
extern Kalman_t kf_pitch;
extern Kalman_t kf_yaw;

static bool accel_trust = true;

AHRS_Offsets_t g_offsets = {0.0f, 0.0f, 0.0f, false};


static float wrap_deg(float a) {
	a = fmodf(a + 180.0f, 360.0f);
	if (a < 0) a += 360.0f;
	return a - 180.0f;
}

float bias_roll = 0, bias_pitch = 0, bias_yaw = 0;

/**
 * @brief Performs sensor fusion to update the global VehicleState.
 * Replicates logic from main.c 100Hz loop.
 */
void AHRS_Update(ahrsSensor_t* raw, vehicleState_t* state, float dt)
{

	if (dt < 0.001f) dt = 0.001f;
	if (dt > 0.010f) dt = 0.010f;

	float gx =  -raw->gy - g_offsets.roll_bias;   // body X
	float gy =  raw->gx - g_offsets.pitch_bias;  // body Y
	float gz =  raw->gz - g_offsets.yaw_bias;    // body Z (no flip)

	float ax = raw->ax;
	float ay = raw->ay;
	float az = raw->az;

	Kalman_Predict(&kf_roll,  gx, dt);
	Kalman_Predict(&kf_pitch, gy, dt);
	Kalman_Predict(&kf_yaw,   gz, dt);

	/* -------------------------------------------------
	 * 2. Accelerometer observation (DO NOT TOUCH)
	 * ------------------------------------------------- */
	float accel_roll  = atan2f(-ay, az) * 57.29578f;
	float accel_pitch = atan2f( ax, sqrtf(ay*ay + az*az)) * 57.29578f;

	float a_mag = sqrtf(raw->ax*raw->ax + raw->ay*raw->ay + raw->az*raw->az);

	// Hysteresis latch
	static bool accel_trust = true;
	float amag_err = fabsf(a_mag - 1.0f);
	if (accel_trust) {
		if (amag_err > 0.25f) accel_trust = false;
	} else {
		if (amag_err < 0.15f) accel_trust = true;
	}

	/* -------------------------------------------------
	 * 3. Magnetometer (tilt compensated yaw)
	 * ------------------------------------------------- */
	float phi   = kf_roll.angle  * 0.0174533f;
	float theta = kf_pitch.angle * 0.0174533f;

	float mx = raw->mx - 0.24f;
	float my = raw->my - 0.24f;
	float mz = raw->mz + 0.08f;

	float By = my * cosf(phi) - mz * sinf(phi);
	float Bx = mx * cosf(theta) +
			(my * sinf(phi) + mz * cosf(phi)) * sinf(theta);

	float mag_yaw = atan2f(-By, Bx) * 57.29578f;

	while (mag_yaw > 180.0f) mag_yaw -= 360.0f;
	while (mag_yaw < -180.0f) mag_yaw += 360.0f;

	/* -------------------------------------------------
	 * 4. Kalman updates
	 * ------------------------------------------------- */
    if (accel_trust) {
        state->roll  = Kalman_Update(&kf_roll,  accel_roll);
        state->pitch = Kalman_Update(&kf_pitch, accel_pitch);
    } else {
        state->roll  = kf_roll.angle;
        state->pitch = kf_pitch.angle;
    }
    float yaw_pred = kf_yaw.angle;
    float yaw_err  = wrap_deg(mag_yaw - yaw_pred);
    float yaw_meas = yaw_pred + yaw_err;
    state->yaw = Kalman_Update(&kf_yaw, yaw_meas);

	/* -------------------------------------------------
	 * 5. Body rates (rad/s)
	 * ------------------------------------------------- */
	state->gyro_x = gx * 0.0174533f; // converting deg/s to rad/s
	state->gyro_y = gy * 0.0174533f;
	state->gyro_z = gz * 0.0174533f;

	state->roll  = -state->roll;
	state->pitch = -state->pitch;
	// Also map to your rate members for the PID
	state->roll_rate  = gx;
	state->pitch_rate = gy;
	state->yaw_rate   = gz;
}


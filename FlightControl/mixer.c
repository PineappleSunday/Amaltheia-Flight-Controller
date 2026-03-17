#include "mixer.h"
#include <stdint.h>
#include <math.h>
#include "main.h"

#define MOTOR_SCALE_MIN 0.1f
#define MOTOR_SCALE_MAX 3.0f

static float g_motor_scale[4] = {1.0f, 1.0f, 0.8f, 1.1f};

static float constrain(float value, float min, float max) {
	if (value < min) return min;
	if (value > max) return max;
	return value;
}

// Constraints based on your PhysicalConstants (max_rpm/max_w_sq)
#define PWM_MIN 1100 // 1.1ms
#define PWM_MAX 1900 // 1.9ms (95% safety margin per your Python model)

void Mixer_ResetMotorScales(void) {
	g_motor_scale[0] = 1.0f;
	g_motor_scale[1] = 1.0f;
	g_motor_scale[2] = 1.0f;
	g_motor_scale[3] = 1.0f;
}

void Mixer_SetMotorScale(uint8_t motor_index, float scale) {
	if (motor_index >= 4) return;
	g_motor_scale[motor_index] = constrain(scale, MOTOR_SCALE_MIN, MOTOR_SCALE_MAX);
}

void Mixer_SetMotorScales(const float scales[4]) {
	if (!scales) return;
	for (uint8_t i = 0; i < 4; i++) {
		g_motor_scale[i] = constrain(scales[i], MOTOR_SCALE_MIN, MOTOR_SCALE_MAX);
	}
}

void Mixer_GetMotorScales(float scales[4]) {
	if (!scales) return;
	for (uint8_t i = 0; i < 4; i++) {
		scales[i] = g_motor_scale[i];
	}
}

static float apply_thrust_linearization(float input_percent) {
	// Linearization expects percent-domain input (0..100).
	if (input_percent <= 0.0f) return 0.0f;
	if (input_percent >= 100.0f) return 100.0f;

	const float expo = 0.55f;
	float x = input_percent / 100.0f;
	float y = x * (1.0f + expo * (x - 1.0f));
	y = constrain(y, 0.0f, 1.0f);
	return y * 100.0f;
}

uint8_t Mixer_Apply(float thrust, float roll, float pitch, float yaw, float* motor_percentages) {
	uint8_t sat = 0;
	// Logic for Plus (+) Configuration
	float m1_raw = thrust + pitch + yaw;
	float m2_raw = thrust - roll - yaw;
	float m3_raw = thrust - pitch + yaw;
	float m4_raw = thrust + roll - yaw;

	float m1 = apply_thrust_linearization(m1_raw) * g_motor_scale[0]; // Front (CCW)
	float m2 = apply_thrust_linearization(m2_raw) * g_motor_scale[1]; // Right (CW)
	float m3 = apply_thrust_linearization(m3_raw) * g_motor_scale[2]; // Rear (CCW)
	float m4 = apply_thrust_linearization(m4_raw) * g_motor_scale[3]; // Left (CW)

	// Low-side saturation uses pre-linearization mixed command to preserve negative detection.
	if (m1_raw <= 0.0f) sat |= (1 << 0);
	if (m2_raw <= 0.0f) sat |= (1 << 1);
	if (m3_raw <= 0.0f) sat |= (1 << 2);
	if (m4_raw <= 0.0f) sat |= (1 << 3);

	// High-side saturation uses post-scale values to match final 90% clamp boundary.
	if (m1 >= 90.0f) sat |= (1 << 4);
	if (m2 >= 90.0f) sat |= (1 << 5);
	if (m3 >= 90.0f) sat |= (1 << 6);
	if (m4 >= 90.0f) sat |= (1 << 7);

	motor_percentages[0] = constrain(m1, 20.0f, 90.0f);
	motor_percentages[1] = constrain(m2, 20.0f, 90.0f);
	motor_percentages[2] = constrain(m3, 20.0f, 90.0f);
	motor_percentages[3] = constrain(m4, 20.0f, 90.0f);

	return sat;

}

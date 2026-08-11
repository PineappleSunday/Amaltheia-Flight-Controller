#include "mixer.h"
#include <stdint.h>
#include <math.h>
#include "main.h"

#define MOTOR_SCALE_MIN 0.1f
#define MOTOR_SCALE_MAX 3.0f

/*
 * PLUS AIRCRAFT CONFIGURATION
 *
 * Operational aircraft frame:
 *
 *            M1 Front (+X)
 *
 * M4 Left (-Y)        M2 Right (+Y)
 *
 *            M3 Rear (-X)
 *
 * Inputs roll/pitch/yaw are aircraft-frame torque commands,
 * NOT raw AHRS/FC-frame quantities.
 *
 * +Roll  -> M4 increase / M2 decrease
 * +Pitch -> M1 increase / M3 decrease
 */

static float g_motor_scale[4] = {1.0f, 1.0f, 1.0f, 1.0f};
static MixerConfig g_mixer_config = {
	.frame_type = MIXER_DEFAULT_FRAME_TYPE
};

static float constrain(float value, float min, float max) {
	if (value < min) return min;
	if (value > max) return max;
	return value;
}

// Constraints based on your PhysicalConstants (max_rpm/max_w_sq)
#define PWM_MIN 1100 // 1.1ms 
#define PWM_MAX 1900 // 1.9ms (90% safety margin per your Python model)

void Mixer_SetConfig(const MixerConfig* config) {
	if (!config) return;
	Mixer_SetFrameType(config->frame_type);
}

void Mixer_GetConfig(MixerConfig* config) {
	if (!config) return;
	config->frame_type = g_mixer_config.frame_type;
}

void Mixer_SetFrameType(MixerFrameType frame_type) {
	if (frame_type != MIXER_FRAME_PLUS && frame_type != MIXER_FRAME_X) {
		return;
	}
	g_mixer_config.frame_type = frame_type;
}

MixerFrameType Mixer_GetFrameType(void) {
	return g_mixer_config.frame_type;
}

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

static void Mixer_ComputeRaw(MixerFrameType frame_type, float thrust, float roll, float pitch, float yaw,
		float* m1_raw, float* m2_raw, float* m3_raw, float* m4_raw) {
	if (!m1_raw || !m2_raw || !m3_raw || !m4_raw) return;

	if (frame_type == MIXER_FRAME_X) {
		// X configuration motor order:
		// M1=FrontRight, M2=RearRight, M3=RearLeft, M4=FrontLeft
		// Roll/pitch diagonal contribution uses 1/sqrt(2).
		// Yaw follows existing spin convention by channel:
		// M1/M3 CCW (+yaw), M2/M4 CW (-yaw).
		const float k = 0.70710678f; // 1/sqrt(2)
		*m1_raw = thrust + k * (pitch - roll) + yaw;
		*m2_raw = thrust + k * (-pitch - roll) - yaw;
		*m3_raw = thrust + k * (-pitch + roll) + yaw;
		*m4_raw = thrust + k * (pitch + roll) - yaw;
		return;
	}

	// Plus (+) configuration motor order:
	// M1=Front, M2=Right, M3=Rear, M4=Left
	*m1_raw = thrust + pitch + yaw;
	*m2_raw = thrust - roll - yaw;
	*m3_raw = thrust - pitch + yaw;
	*m4_raw = thrust + roll - yaw;
}

uint8_t Mixer_Apply(float thrust, float roll, float pitch, float yaw, float* motor_percentages) {
	if (!motor_percentages) return 0;

	uint8_t sat = 0;
	float m1_raw = 0.0f, m2_raw = 0.0f, m3_raw = 0.0f, m4_raw = 0.0f;
	Mixer_ComputeRaw(g_mixer_config.frame_type, thrust, roll, pitch, yaw, &m1_raw, &m2_raw, &m3_raw, &m4_raw);

	float m1 = apply_thrust_linearization(m1_raw) * g_motor_scale[0];
	float m2 = apply_thrust_linearization(m2_raw) * g_motor_scale[1];
	float m3 = apply_thrust_linearization(m3_raw) * g_motor_scale[2];
	float m4 = apply_thrust_linearization(m4_raw) * g_motor_scale[3];

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

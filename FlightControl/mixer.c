#include "mixer.h"
#include <stdint.h>
#include <math.h>
#include "main.h"

static float constrain(float value, float min, float max) {
	if (value < min) return min;
	if (value > max) return max;
	return value;
}

// Constraints based on your PhysicalConstants (max_rpm/max_w_sq)
#define PWM_MIN 1100 // 1.1ms
#define PWM_MAX 1900 // 1.9ms (95% safety margin per your Python model)


void Mixer_Apply(float thrust, float roll, float pitch, float yaw, float* motor_percentages) {
	// Logic for Plus (+) Configuration
	float m1 = thrust + pitch + yaw; // Front
	float m2 = thrust - roll - yaw;  // Right
	float m3 = thrust - pitch + yaw; // Rear
	float m4 = thrust + roll - yaw;  // Left

	motor_percentages[0] = constrain(m1, 0.0f, 90.0f);
	motor_percentages[1] = constrain(m2, 0.0f, 90.0f);
	motor_percentages[2] = constrain(m3, 0.0f, 90.0f);
	motor_percentages[3] = constrain(m4, 0.0f, 90.0f);

}

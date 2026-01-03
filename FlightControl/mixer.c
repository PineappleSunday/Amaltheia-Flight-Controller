#include "mixer.h"
#include <stdint.h>
#include <math.h>

static float constrain(float value, float min, float max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

// Constraints based on your PhysicalConstants (max_rpm/max_w_sq)
#define PWM_MIN 1100 // 1.1ms
#define PWM_MAX 1900 // 1.9ms (95% safety margin per your Python model)

float Mixer_Apply(float thrust, float roll, float pitch, float yaw, uint32_t* motor_outputs) {
    // Standard Quad X Mixing logic from your quad_model_runnion.py
    float m1 = thrust + pitch + yaw; // Front
    float m2 = thrust - roll - yaw;  // Right
    float m3 = thrust - pitch + yaw; // Rear
    float m4 = thrust + roll - yaw;  // Left

    motor_outputs[0] = (uint32_t)constrain(m1, PWM_MIN, PWM_MAX);
    motor_outputs[1] = (uint32_t)constrain(m2, PWM_MIN, PWM_MAX);
    motor_outputs[2] = (uint32_t)constrain(m3, PWM_MIN, PWM_MAX);
    motor_outputs[3] = (uint32_t)constrain(m4, PWM_MIN, PWM_MAX);
}
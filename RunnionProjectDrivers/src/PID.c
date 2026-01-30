// Arbiter Industries Inc.
// The arbiter of defensive technologies
// Author: C.RUNNION
// PID Control
// References:
// https://timhanewich.medium.com/how-i-developed-the-scout-flight-controller-part-4-stabilizing-flight-with-pid-controllers-1e945577a9aa
// https://pidexplained.com/pid-controller-explained/

#include "PID.h"

void PID_Init(PIDController* pid, float kp, float ki, float kd, float cycle_time_seconds, float i_limit) {
    if (!pid) return;

    // Settings
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->cycle_time_seconds = cycle_time_seconds;
    pid->i_limit = i_limit;

    // Reset state
    PID_Reset(pid);
}

float PID_Calculate(PIDController* pid, float actual, float goal) {
    if (!pid) return 0.0f;

    float error = goal - actual;

    // 1. Proportional term
    float P = error * pid->kp;

    // 2. Integral term with anti-windup
    float I = pid->previous_i + (error * pid->ki * pid->cycle_time_seconds);

    // Clamp I-term
    if (I > pid->i_limit) I = pid->i_limit;
    else if (I < -pid->i_limit) I = -pid->i_limit;

    // Safety: Prevent divide-by-zero
    if (pid->cycle_time_seconds <= 0.0f) {
        pid->cycle_time_seconds = 0.002f; // Default to 500Hz if unset
    }

    // 3. Derivative term calculation
    // Calculate the raw derivative first
    float D_raw = pid->kd * (error - pid->previous_error) / pid->cycle_time_seconds;
    float D_final = D_raw;

    // Apply Low Pass Filter if alpha is set (0.0 to 1.0)
    if (pid->d_low_pass_alpha > 0.0f && pid->d_low_pass_alpha < 1.0f) {
        D_final = (pid->d_low_pass_alpha * D_raw) +
                  ((1.0f - pid->d_low_pass_alpha) * pid->previous_d_filtered);
    }

    // 4. Calculate Total Output
    float total_output = P + I + D_final;

    // 5. Store State for Next Loop
    pid->previous_error = error;
    pid->previous_i = I;
    pid->previous_d_filtered = D_final; // Store filtered value for next iteration

    // 6. Store Terms for Logging
    pid->p_out = P;
    pid->i_out = I;
    pid->d_out = D_final;
    pid->output = total_output;

    return total_output;
}

float PID_Calculate_Alpha(float cutoff_hz, float loop_time_s) {
    float tau = 1.0f / (2.0f * 3.14159f * cutoff_hz);
    float alpha = loop_time_s / (tau + loop_time_s);

    // Clamp alpha between 0 and 1
    if (alpha > 1.0f) alpha = 1.0f;
    if (alpha < 0.0f) alpha = 0.0f;

    return alpha;
}

void PID_Reset(PIDController* pid) {
    if (!pid) return;
    pid->previous_error = 0.0f;
    pid->previous_i = 0.0f;
}

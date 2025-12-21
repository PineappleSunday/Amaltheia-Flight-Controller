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

    // Proportional term
    float P = error * pid->kp;

    // Integral term with anti-windup
    float I = pid->previous_i + (error * pid->ki * pid->cycle_time_seconds);
    if (I > pid->i_limit) I = pid->i_limit;
    else if (I < -pid->i_limit) I = -pid->i_limit;

    // Derivative term
    float D = pid->kd * (error - pid->previous_error) / pid->cycle_time_seconds;

    // Store state for the next iteration
    pid->previous_error = error;
    pid->previous_i = I;

    return P + I + D;
}

void PID_Reset(PIDController* pid) {
    if (!pid) return;
    pid->previous_error = 0.0f;
    pid->previous_i = 0.0f;
}

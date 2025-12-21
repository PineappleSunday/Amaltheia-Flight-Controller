#pragma once

// Arbiter Industries Inc.
// The arbiter of defensive technologies
// Author: C.RUNNION
// PID Control
// References:
// https://timhanewich.medium.com/how-i-developed-the-scout-flight-controller-part-4-stabilizing-flight-with-pid-controllers-1e945577a9aa
// https://pidexplained.com/pid-controller-explained/

#include <stdint.h>

/**
 * @brief Basic PID controller implementation.
 *
 * Math modeled based on guide found here: https://pidexplained.com/pid-controller-explained/
 */
typedef struct {
    // Settings
    float kp, ki, kd;
    float cycle_time_seconds;
    float i_limit;

    // State variables
    float previous_error;
    float previous_i;
} PIDController;

/**
 * @brief Initializes a PIDController struct.
 * @param pid Pointer to the PIDController struct.
 * @param kp Proportional gain.
 * @param ki Integral gain.
 * @param kd Derivative gain.
 * @param cycle_time_seconds The PID cycle time in seconds (e.g., 0.004 for 250Hz).
 * @param i_limit Anti-windup limit for the integral term.
 */
void PID_Init(PIDController* pid, float kp, float ki, float kd, float cycle_time_seconds, float i_limit);

/**
 * @brief Performs a PID calculation.
 * @param pid Pointer to the PIDController struct.
 * @param actual The current reading from the sensor.
 * @param goal The desired setpoint.
 * @return The calculated PID output.
 */
float PID_Calculate(PIDController* pid, float actual, float goal);

/** @brief Resets the controller's internal state variables. */
void PID_Reset(PIDController* pid);

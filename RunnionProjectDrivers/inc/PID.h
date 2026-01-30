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
    // --- Configuration (Gains) ---
    float kp;
    float ki;
    float kd;

    // --- Timing & Safety ---
    float cycle_time_seconds; // The fixed loop interval (e.g., 0.002f for 500Hz)
    float i_limit;            // Maximum value for the integrated error (Anti-Windup)

    // --- Noise Filtering (LPF) ---
    float d_low_pass_alpha;   // Alpha value for 1st order D-term LPF (0.0 = Off)

    // --- Internal State (Memory) ---
    float previous_error;      // Required for the Derivative term
    float previous_i;          // Required for the Integral term
    float previous_d_filtered; // Required for the Low Pass Filter iteration

    // --- Telemetry & Debugging (Logging) ---
    // These allow you to see which term is causing motor heat or oscillation
    float p_out;               // Last calculated Proportional contribution
    float i_out;               // Last calculated Integral contribution
    float d_out;               // Last calculated Derivative contribution (filtered)
    float output;              // Final summed output
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

/**
 * @brief Helper to calculate Alpha for the D-term filter based on frequency.
 */
float PID_Calculate_Alpha(float cutoff_hz, float loop_time_s);

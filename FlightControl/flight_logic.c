#include "flight_logic.h"
#include "PID.h"
#include "mixer.h"
#include "state.h"
#include "main.h" // Essential for ESC_SetThrottle and get_timer_channel

// External PID instances from main.c
extern PIDController pid_roll, pid_pitch, pid_yaw;
// Assuming you add velocity/position PIDs to your main.c globals
extern PIDController pid_pos_z, pid_vel_z;
static const float ESC_MIN_VAL = 1000.0f;

void FlightLogic_Update(vehicleState_t* state, targetState_t* target) {
    // 1. ALTITUDE CONTROL (Cascaded)
    float corr_vel_z = PID_Calculate(&pid_pos_z, state->z, target->z);
    float target_vz  = corr_vel_z + target->ff_vz;
    float thrust_adj = PID_Calculate(&pid_vel_z, state->vz, target_vz);

    // 1500us is a typical starting hover point for a 4S build
    float base_thrust = 1500.0f + thrust_adj;

    // 2. ATTITUDE CONTROL (Inner Loop)
    float roll_cmd  = PID_Calculate(&pid_roll,  state->roll,  target->roll);
    float pitch_cmd = PID_Calculate(&pid_pitch, state->pitch, target->pitch);
    float yaw_cmd   = PID_Calculate(&pid_yaw,   state->yaw,   target->yaw);

    // 3. MOTOR MIXING
    uint32_t motor_pwms[4];
    Mixer_Apply(base_thrust, roll_cmd, pitch_cmd, yaw_cmd, motor_pwms);

    // 4. HARDWARE ACTUATION
    for (int i = 0; i < 4; i++) {
        // Fix: Use 'motor_pwms' to match the declaration above
        // Convert Pulse (1000-2000) to Percentage (0-100) for ESC_SetThrottle
        float pct = (float)(motor_pwms[i] - ESC_MIN_VAL) / 10.0f;

        // Scope is provided by main.h
        ESC_SetThrottle(get_timer_channel(i + 1), pct);
    }
}

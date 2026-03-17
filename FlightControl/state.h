// state.h
#ifndef STATE_H     
#define STATE_H

#include <stdint.h>
#include <stdbool.h>

#define MODE_MANUAL_LEVEL  1  // Auto-level on, but user controls vertical
#define MODE_MISSION       2  // Following your mission_plan[64] string
#define MODE_THRUST_STAND  3
// Mode ID 4 is reserved/unsupported (obsolete AHRS-only mode)

#define SUBMODE_STABLE     0
#define SUBMODE_TAKEOFF    1

// Kinematic State of the drone (Single Source of Truth)
typedef struct {
    float x, y, z;
    float vx, vy, vz;
    float roll, pitch, yaw;
    float q0, q1, q2, q3;
    // Add these for the Rate PID
    float gyro_x, gyro_y, gyro_z;
    float roll_rate, pitch_rate, yaw_rate;
    bool offGround;
    bool isTuning;
    float dt_sec;
} vehicleState_t;

// Desired State (from Navigation/Mission Manager)
typedef struct {
    float x, y, z, yaw;
    float roll, pitch;         // Target Angles
    float rate_roll, rate_pitch, rate_yaw; // Target Rates
    float ff_vx, ff_vy, ff_vz;
    bool yaw_hold_enabled;
} targetState_t;

// Drone Status and Telemetry Metadata
typedef struct {
    char mission_plan[64];
    uint8_t drone_mode;
    uint8_t drone_submode;
    uint8_t motor1_T, motor2_T, motor3_T, motor4_T;
    uint8_t flight_mode;
    uint8_t armed;
    uint8_t low_battery_warning;
    float battery_voltage;
} droneState_t;

#endif // STATE_H

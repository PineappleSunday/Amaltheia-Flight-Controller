// state.h
#ifndef STATE_H     
#define STATE_H

#include <stdint.h>

// Kinematic State of the drone (Single Source of Truth)
typedef struct {
    float x, y, z;          // Position [m]
    float vx, vy, vz;       // Velocity [m/s]
    float roll, pitch, yaw; // Attitude [rad]
    float p, q, r;          // Body rates [rad/s]
} vehicleState_t;

// Desired State (from Navigation/Mission Manager)
typedef struct {
    float x, y, z, yaw;        // Target position/heading
    float roll, pitch;         // Target attitude (calculated from position PIDs)
    float ff_vx, ff_vy, ff_vz; // Feedforward velocities
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

// state.c

#include "state.h"
#include <string.h>

void Vehicle_State_Init(VehicleState* state) {
    memset(state, 0, sizeof(VehicleState));
}

void Target_State_Init(TargetState* target) {
    memset(target, 0, sizeof(TargetState));
}


typedef struct {
    float x, y, z;          // Position [m]
    float vx, vy, vz;       // Velocity [m/s]
    float roll, pitch, yaw; // Attitude [rad]
    float p, q, r;          // Body rates [rad/s]
} VehicleState;

typedef struct {
    float x, y, z, yaw;     // Target setpoints
    float ff_vx, ff_vy, ff_vz; // Feedforward velocities [m/s]
} TargetState;
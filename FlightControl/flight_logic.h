// flight_logic.h
// 
#ifndef FLIGHT_LOGIC_H
#define FLIGHT_LOGIC_H

#include "state.h"
#include "navigation.h"
#include "pid.h"
#include "mixer.h"

// The "Brain" of the drone that holds all controller instances
typedef struct {
    PIDController pid_roll;
    PIDController pid_pitch;
    PIDController pid_yaw;
    PIDController pid_alt;
    // Add velocity/position PIDs here as you scale
} FlightControlStack;

void FlightLogic_Init(void);

// Replaces updateFlightLogic to align with 100Hz timing requirements
void FlightLogic_Update(vehicleState_t* state, targetState_t* target);

#endif

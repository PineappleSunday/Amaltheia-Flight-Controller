// state.c

#include "state.h"
#include <string.h>

// Using the typedef names from your header
void Vehicle_State_Init(droneState_t* state) {
    if (state == NULL) return;
    memset(state, 0, sizeof(droneState_t));
    state->armed = 0; // Explicit safety zero
}

void Target_State_Init(targetState_t* target) {
    memset(target, 0, sizeof(targetState_t));
}

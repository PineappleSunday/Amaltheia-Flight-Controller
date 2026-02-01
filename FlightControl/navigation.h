// navigation.h

#include <stdint.h>
#include "state.h"
#ifndef NAVIGATE_H
#define NAVIGATE_H

struct vehicleState_t;
struct targetState_t;


typedef enum {
    WP_ACTION_TAKEOFF,
    WP_ACTION_HOVER,      
    WP_ACTION_LAND,
    WP_ACTION_MOVE
} WaypointAction;

typedef struct {
    float position[4];      // [x, y, z, yaw]
    float toa;              // Time of Arrival [s]
    float duration;         // Hover duration [s]
    float tolerance;        // Arrival radius [m]
    WaypointAction action;
} Waypoint;

typedef struct {
    Waypoint* waypoints;
    char mission_name[64];
    uint16_t current_index;
    float hover_start_time; // t when hover started
    float prev_dist;
    float wp_start_time;    // t when current leg started
    // [x, y, z, yaw] [m, rad]
    float prev_wp_pos[4];   // Start position of current leg
    uint16_t total_waypoints;
    float landing_start_t;  // Timestamp for descent logic
    uint8_t is_complete;
} MissionManager;

void Navigation_GetTarget(MissionManager* mgr, float t, const vehicleState_t* state, targetState_t* target);

#endif // NAVIGATE_H (This closes the #ifndef block)

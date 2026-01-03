// navigate.c
#include "navigation.h"
#include "state.h"
#include <math.h>
#include <float.h>

extern droneState_t g_drone_status;

static void Navigation_HandleArrival(MissionManager* mgr, float t, Waypoint* wp);
void Navigation_Advance(MissionManager* mgr, float t);
void Navigation_GetTarget(MissionManager* mgr, float t, const vehicleState_t* state, targetState_t* target);

void Navigation_GetTarget(MissionManager* mgr, float t, const vehicleState_t* state, targetState_t* target) {
    // 1. Check if mission is complete
    if (mgr->current_index >= mgr->total_waypoints) {
        Waypoint* last_wp = &mgr->waypoints[mgr->total_waypoints - 1];
        target->x = last_wp->position[0];
        target->y = last_wp->position[1];
        target->z = last_wp->position[2];
        target->yaw = last_wp->position[3];
        target->ff_vx = target->ff_vy = target->ff_vz = 0.0f;
        return;
    }

    Waypoint* wp = &mgr->waypoints[mgr->current_index];
    float leg_duration = wp->toa - mgr->wp_start_time;

    // --- 2. BRANCH: INSTANT JUMP (Zero TOA) vs INTERPOLATION ---
    if (leg_duration <= 0.0f) {
        // Snap directly to target
        target->x = wp->position[0];
        target->y = wp->position[1];
        target->z = wp->position[2];
        target->yaw = wp->position[3];
        target->ff_vx = target->ff_vy = target->ff_vz = 0.0f;
    }
    else {
        // --- 3. BRANCH: SPECIAL LANDING vs PATH INTERPOLATION ---
        if (wp->action == WP_ACTION_LAND && mgr->landing_start_t > 0.0f) {
            float elapsed = t - mgr->landing_start_t;
            float descent_rate;
            g_drone_status.flight_mode = 3;
            // Tiered Descent Rates
            if (state->z <= 0.05f) {
                mgr->is_complete = 1;
                descent_rate = 0.0f;
            } else if (state->z > 20.0f) {
                descent_rate = 0.6f;
            } else if (state->z > 5.0f) {
                descent_rate = 0.4f;
            } else {
                descent_rate = 0.15f;
            }

            target->z = wp->position[2] - (descent_rate * elapsed);
            if (target->z < 0.0f) target->z = 0.0f;

            target->x = wp->position[0];
            target->y = wp->position[1];
            target->yaw = wp->position[3];
            target->ff_vz = -descent_rate;
            target->ff_vx = 0.0f;
            target->ff_vy = 0.0f;

            // Note: Prevent early exit here so distance checks still run
        }
        else {
            // Standard Interpolation
            float pct = (t - mgr->wp_start_time) / leg_duration;
            pct = (pct < 0.0f) ? 0.0f : (pct > 1.0f) ? 1.0f : pct;

            target->x = mgr->prev_wp_pos[0] + (wp->position[0] - mgr->prev_wp_pos[0]) * pct;
            target->y = mgr->prev_wp_pos[1] + (wp->position[1] - mgr->prev_wp_pos[1]) * pct;
            target->z = mgr->prev_wp_pos[2] + (wp->position[2] - mgr->prev_wp_pos[2]) * pct;
            target->yaw = mgr->prev_wp_pos[3] + (wp->position[3] - mgr->prev_wp_pos[3]) * pct;

            if (pct >= 1.0f) {
                target->ff_vx = target->ff_vy = target->ff_vz = 0.0f;
            } else {
                target->ff_vx = (wp->position[0] - mgr->prev_wp_pos[0]) / leg_duration;
                target->ff_vy = (wp->position[1] - mgr->prev_wp_pos[1]) / leg_duration;
                target->ff_vz = (wp->position[2] - mgr->prev_wp_pos[2]) / leg_duration;
            }
        }
    }

    // --- 4. DISTANCE & OVERSHOOT CHECKS ---
    float dx = wp->position[0] - state->x;
    float dy = wp->position[1] - state->y;
    float dz = wp->position[2] - state->z;
    float remaining_dist = sqrtf(dx*dx + dy*dy + dz*dz);

    if (remaining_dist < wp->tolerance) {
        Navigation_HandleArrival(mgr, t, wp);
    } 
    else if ((remaining_dist - mgr->prev_dist) > 0.5f && mgr->hover_start_time <= 0.0f) {
        if (remaining_dist < 5.0f) {
            Navigation_Advance(mgr, t);
        }
    }

    mgr->prev_dist = remaining_dist;
}

static void Navigation_HandleArrival(MissionManager* mgr, float t, Waypoint* wp) {
	if (wp->action == WP_ACTION_HOVER) {
		g_drone_status.flight_mode = 2; // 2 = HOVERING
		if (mgr->hover_start_time <= 0.0f) {
			mgr->hover_start_time = t;
		}
		if (t - mgr->hover_start_time >= wp->duration) {
			Navigation_Advance(mgr, t);
		}
	} else if (wp->action == WP_ACTION_LAND) {
		g_drone_status.flight_mode = 3; // 3 = LANDING
		if (mgr->landing_start_t <= 0.0f) {
			mgr->landing_start_t = t;
		}
	} else {
		g_drone_status.flight_mode = 1; // 1 = MOVING
		Navigation_Advance(mgr, t);
	}
}

void Navigation_Advance(MissionManager* mgr, float t) {
	// Store current WP position as start for next leg
	Waypoint* current_wp = &mgr->waypoints[mgr->current_index];
	for(int i=0; i<4; i++) mgr->prev_wp_pos[i] = current_wp->position[i];
	g_drone_status.flight_mode = 1;
	mgr->current_index++;
	mgr->hover_start_time = 0.0f;
	mgr->prev_dist = FLT_MAX;
	mgr->wp_start_time = t;
}

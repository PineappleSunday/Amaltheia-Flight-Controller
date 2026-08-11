#ifndef SENSOR_FRAMES_H
#define SENSOR_FRAMES_H

typedef struct {
    float x;
    float y;
    float z;
} Vec3f;

/*
 * PCB/sensor frame -> AHRS frame.
 *
 * Calibration established these are presently identical:
 *
 *   FC +X -> M3
 *   FC +Y -> M4
 *   FC +Z -> down
 */
static inline Vec3f SensorFrames_PcbToBody(Vec3f pcb)
{
    return pcb;
}

/*
 * AHRS/FC estimator frame -> operational aircraft control frame.
 *
 * AHRS:
 *   +X -> M3
 *   +Y -> M4
 *
 * Aircraft:
 *   +X -> M1 (front)
 *   +Y -> M2 (right)
 *
 * 180 degree rotation about Z:
 *
 *   X_aircraft = -X_ahrs
 *   Y_aircraft = -Y_ahrs
 *   Z_aircraft = +Z_ahrs
 */
static inline Vec3f SensorFrames_AhrsToAircraft(Vec3f v)
{
    Vec3f out = {
        .x = -v.x,
        .y = -v.y,
        .z =  v.z
    };

    return out;
}

#endif
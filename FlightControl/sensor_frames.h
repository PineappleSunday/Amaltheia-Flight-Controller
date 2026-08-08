#ifndef SENSOR_FRAMES_H
#define SENSOR_FRAMES_H

typedef struct {
	float x;
	float y;
	float z;
} Vec3f;

// Flight Controller +X points toward M3 and Flight Controller +Y points toward M4.
// Those are currently body-forward and body-right, so this transform is identity.
static inline Vec3f SensorFrames_PcbToBody(Vec3f FLCON_PCB)
{
	return FLCON_PCB;
}

#endif // SENSOR_FRAMES_H

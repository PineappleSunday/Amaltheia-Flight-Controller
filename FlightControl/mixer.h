// mixer.h
// Created for Arbiter Technologies' Amaltheia Flight Controller
// Defines the interface for the mixer module responsible for translating

#ifndef MIXER_H
#define MIXER_H

#include <stdint.h>

typedef enum {
	MIXER_FRAME_PLUS = 0,
	MIXER_FRAME_X = 1
} MixerFrameType;

typedef struct {
	MixerFrameType frame_type;
} MixerConfig;

// Build-time default mixer frame.
// Override with compiler define, e.g.:
// -DMIXER_DEFAULT_FRAME_TYPE=MIXER_FRAME_X
#ifndef MIXER_DEFAULT_FRAME_TYPE
#define MIXER_DEFAULT_FRAME_TYPE MIXER_FRAME_PLUS
#endif

void Mixer_ResetMotorScales(void);
void Mixer_SetMotorScale(uint8_t motor_index, float scale);
void Mixer_SetMotorScales(const float scales[4]);
void Mixer_GetMotorScales(float scales[4]);
void Mixer_SetConfig(const MixerConfig* config);
void Mixer_GetConfig(MixerConfig* config);
void Mixer_SetFrameType(MixerFrameType frame_type);
MixerFrameType Mixer_GetFrameType(void);
uint8_t Mixer_Apply(float thrust, float roll, float pitch, float yaw, float* motor_percentages);

#endif // MIXER_H

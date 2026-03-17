// mixer.h
// Created for Arbiter Technologies' Amaltheia Flight Controller
// Defines the interface for the mixer module responsible for translating

#ifndef MIXER_H
#define MIXER_H

#include <stdint.h>

void Mixer_ResetMotorScales(void);
void Mixer_SetMotorScale(uint8_t motor_index, float scale);
void Mixer_SetMotorScales(const float scales[4]);
void Mixer_GetMotorScales(float scales[4]);
uint8_t Mixer_Apply(float thrust, float roll, float pitch, float yaw, float* motor_percentages);

#endif // MIXER_H

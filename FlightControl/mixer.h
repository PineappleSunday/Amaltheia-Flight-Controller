// mixer.h
// Created for Arbiter Technologies' Amaltheia Flight Controller
// Defines the interface for the mixer module responsible for translating

#ifndef MIXER_H
#define MIXER_H

#include <stdint.h>

uint8_t Mixer_Apply(float thrust, float roll, float pitch, float yaw, float* motor_percentages);

#endif // MIXER_H

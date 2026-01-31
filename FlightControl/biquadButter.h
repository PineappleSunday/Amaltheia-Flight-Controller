/*
 * biquadButter.h
 *
 *  Created on: Jan 30, 2026
 *      Author: colin
 */

#ifndef BIQUADBUTTER_H_
#define BIQUADBUTTER_H_

#include <stdint.h>
#include "state.h"
#include <stdbool.h>

typedef struct {
    float a[3]; // Feedback coefficients
    float b[3]; // Feedforward coefficients
    float x[2]; // Input history
    float y[2]; // Output history
} BiquadFilter_t;

float Biquad_Process(BiquadFilter_t* f, float input);
void Biquad_Init(BiquadFilter_t* f);
void Biquad_Set_Lowpass(BiquadFilter_t* f, float cutoff_hz, float sampling_hz);
#endif /* BIQUADBUTTER_H_ */

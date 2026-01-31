/*
 * biquadButter.c
 *
 *  Created on: Jan 30, 2026
 *      Author: colin
 */
#include <math.h>
#include "biquadButter.h"

void Biquad_Init(BiquadFilter_t* f) {
    for(int i=0; i<2; i++) {
        f->x[i] = 0.0f;
        f->y[i] = 0.0f;
    }
}

float Biquad_Process(BiquadFilter_t* f, float input) {
    float output = f->b[0] * input + f->b[1] * f->x[0] + f->b[2] * f->x[1]
                   - f->a[1] * f->y[0] - f->a[2] * f->y[1];

    // Update history
    f->x[1] = f->x[0];
    f->x[0] = input;
    f->y[1] = f->y[0];
    f->y[0] = output;

    return output;
}

void Biquad_Set_Lowpass(BiquadFilter_t* f, float cutoff_hz, float sampling_hz) {
    // 1. Calculate warped angular frequency
    float K = tanf(M_PI * cutoff_hz / sampling_hz);
    float K2 = K * K;

    // 2. Q factor for a Butterworth filter (Flat response)
    float Q = 0.7071f;

    // 3. Calculate normalization factor (a0)
    float norm = 1.0f / (1.0f + (K / Q) + K2);

    // 4. Set Feedforward (b) coefficients
    f->b[0] = K2 * norm;
    f->b[1] = 2.0f * f->b[0];
    f->b[2] = f->b[0];

    // 5. Set Feedback (a) coefficients
    f->a[1] = 2.0f * (K2 - 1.0f) * norm;
    f->a[2] = (1.0f - (K / Q) + K2) * norm;

    // 6. Reset history to prevent startup "kicks"
    f->x[0] = f->x[1] = 0.0f;
    f->y[0] = f->y[1] = 0.0f;
}

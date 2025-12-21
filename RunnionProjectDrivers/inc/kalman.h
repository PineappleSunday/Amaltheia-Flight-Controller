/*
 * kalman.h
 *
 *  Created on: Dec 15, 2025
 *      Author: colin
 * 1D Kalman Filter for Attitude Estimation
 */

/*
 * kalman.h
 * 1D Kalman Filter for Attitude Estimation
 */

#ifndef KALMAN_H
#define KALMAN_H

typedef struct {
    float angle;  // The state estimate
    float P;      // Error covariance
    float Q;      // Process noise (Trust in Gyro)
    float R;      // Measurement noise (Trust in Accel/Mag)
} Kalman_t;

// Initialize the filter
void Kalman_Init(Kalman_t *k, float Q, float R);

// Predict step (Gyro Integration)
void Kalman_Predict(Kalman_t *k, float gyro_rate, float dt);

// Update step (Accel/Mag Correction)
float Kalman_Update(Kalman_t *k, float measured_angle);

#endif

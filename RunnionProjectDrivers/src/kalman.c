/*
 * kalman.c
 *
 *  Created on: Dec 15, 2025
 *      Author: colin
 */
#include "kalman.h"

void Kalman_Init(Kalman_t *k, float Q, float R) {
    k->angle = 0.0f; // Reset angle
    k->P = 1.0f;     // Default covariance
    k->Q = Q;
    k->R = R;
}

void Kalman_Predict(Kalman_t *k, float gyro_rate, float dt) {
    /*
     * Python: self.angle += gyro_rate * dt
     *         self.P += self.Q * dt
     */
    k->angle += gyro_rate * dt;
    k->P += k->Q * dt;
}

float Kalman_Update(Kalman_t *k, float measured_angle) {
    /*
     * 1. Calculate Error
     * Python: error = measured_angle - self.angle
     */
    float error = measured_angle - k->angle;

    /*
     * 2. Wrap Error (Handling the -180 to 180 crossover)
     * Python: error = (error + 180) % 360 - 180
     * In C, we use while loops to handle wrap-around efficiently
     */
    while (error > 180.0f)  error -= 360.0f;
    while (error < -180.0f) error += 360.0f;

    /*
     * 3. Calculate Gain
     * Python: K = self.P / (self.P + self.R)
     */
    float K = k->P / (k->P + k->R);

    /*
     * 4. Update State
     * Python: self.angle += K * error
     */
    k->angle += K * error;

    /*
     * 5. Update Covariance
     * Python: self.P = (1 - K) * self.P
     */
    k->P = (1.0f - K) * k->P;

    /*
     * 6. Wrap Output
     * Ensure the internal state stays within -180 to 180
     */
    while (k->angle > 180.0f)  k->angle -= 360.0f;
    while (k->angle < -180.0f) k->angle += 360.0f;

    return k->angle;
}

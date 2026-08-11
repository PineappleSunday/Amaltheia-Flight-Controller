// AHRS.h


#ifndef AHRS_H
#define AHRS_H

#include <stdint.h>
#include "state.h"
#include <stdbool.h>

#ifndef AHRS_MADGWICK_BETA
#define AHRS_MADGWICK_BETA 0.04f
#endif

/**
 * @brief Raw and processed sensor data from the IMU.
 */
typedef struct {    
    // Accelerometer (in g's)
    float ax, ay, az;
    
    // Magnetometer (in Gauss)
    float mx, my, mz;
    
    // Gyroscope (in degrees/sec or rad/sec)
    float gx, gy, gz;

    // Temperature (optional, for sensor compensation)
    float temperature;
} ahrsSensor_t;

// Function prototypes
void AHRS_Init(void);
void AHRS_Update(ahrsSensor_t* raw_data, vehicleState_t* state, float dt);
void AHRS_SetBeta(float beta);

#endif // AHRS_H

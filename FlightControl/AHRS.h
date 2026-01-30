// AHRS.h


#ifndef AHRS_H
#define AHRS_H

#include <stdint.h>
#include "state.h"
#include <stdbool.h>

typedef struct {
    float roll_bias;
    float pitch_bias;
    float yaw_bias;
    bool is_calibrated;
} AHRS_Offsets_t;

// This tells main.c "There is a variable named g_offsets defined somewhere else"
extern AHRS_Offsets_t g_offsets;
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

#endif // AHRS_H

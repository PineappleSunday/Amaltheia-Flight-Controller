// Arbiter Industries Inc. 
// The arbiter of defensive technologies

// Author: C.RUNNION
// LSM303 driver supporting both DLHC and AGR variants
// Supports accelerometer and magnetometer reading (and temperature for AGR)
// Uses I2C HAL interface
// References:
// LSM303DLHC datasheet: https://www.st.com/resource/en/datasheet/lsm303dlhc.pdf
// LSM303AGR datasheet: https://www.st.com/resource/en/datasheet/

#pragma once
#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

typedef struct {
    int16_t ax, ay, az;
    int16_t mx, my, mz;
} LSM303_Raw;

/* Enum to distinguish between LSM303DLHC and LSM303AGR variants

LSM303DLHC:

Older model (now discontinued)
14-bit ADC for accelerometer
±2g/±4g/±8g/±16g acceleration ranges
±1.3 to ±8.1 gauss magnetic field ranges
I2C addresses: 0x19 (accel) and 0x1E (mag)
3.3V operation voltage
Higher power consumption
Different register map from AGR
LSM303AGR:

Newer model (active)
10-bit ADC for accelerometer
±2g/±4g/±8g/±16g acceleration ranges
±50 gauss magnetic field range
I2C addresses: 0x19 (accel) and 0x1E (mag)
Ultra-low power consumption (down to 1μA)
1.71V to 3.6V operation voltage
Better temperature performance
Different register addresses and settings from DLHC
Built-in FIFO
Temperature sensor included
More advanced power management features */
typedef enum {
    LSM303_UNKNOWN = 0,
    LSM303_DLHC,
    LSM303_AGR
} LSM303_Variant;

typedef enum {
    LSM303_ACCEL_SCALE_2G = 0, // default
    LSM303_ACCEL_SCALE_4G,
    LSM303_ACCEL_SCALE_8G,
    LSM303_ACCEL_SCALE_16G
} LSM303_AccelScale;

typedef struct {
    I2C_HandleTypeDef* hi2c;
    // I2C 7-bit addresses (default SA0 = 1 for accel; mag depends on variant)
    uint8_t addr_acc;   // usually 0x19
    uint8_t addr_mag;   // DLHC: 0x1E, AGR: 0x1E or 0x1C depending on SA1
    LSM303_Variant variant;

    // scale factors (filled on init)
    float accel_g_per_lsb;   // g/LSB
    float mag_gauss_per_lsb; // Gauss/LSB
} LSM303;

bool LSM303_Init(LSM303* dev, I2C_HandleTypeDef* hi2c, LSM303_AccelScale scale);
bool LSM303_ReadAccel(LSM303* dev, LSM303_Raw* out);
bool LSM303_ReadMag(LSM303* dev, LSM303_Raw* out);
bool LSM303_ReadTemp(LSM303* dev, int16_t* out);
bool LSM303_ReadTempC(LSM303* dev, float* out_c);
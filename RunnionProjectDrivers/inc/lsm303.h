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
    LSM303_DLHC,
    LSM303_AGR,
    LSM303_UNKNOWN
} LSM303_Variant;

// --- NEW: State Machine Enum ---
typedef enum {
    LSM303_IDLE,
    LSM303_ACCEL_TX,
    LSM303_ACCEL_RX,
    LSM303_MAG_TX,
    LSM303_MAG_RX
} LSM303_State;

// --- Updated Struct ---
typedef struct {
    I2C_HandleTypeDef* hi2c;
    uint8_t addr_acc;
    uint8_t addr_mag;
    LSM303_Variant variant;
    float accel_g_per_lsb;
    float mag_gauss_per_lsb;

    // DMA STATE MACHINE MEMBERS
    LSM303_State state;         // Tracks the current I2C phase
    uint8_t accel_raw[6];       // Buffer for DMA to fill
    uint8_t mag_raw[6];         // Buffer for DMA to fill
    volatile bool accel_ready;  // Flag for main loop
    volatile bool mag_ready;    // Flag for main loop
} LSM303;
typedef enum {
    LSM303_ACCEL_SCALE_2G,
    LSM303_ACCEL_SCALE_4G,
    LSM303_ACCEL_SCALE_8G,
    LSM303_ACCEL_SCALE_16G
} LSM303_AccelScale;
// --- Prototypes ---
bool LSM303_Init(LSM303* dev, I2C_HandleTypeDef* hi2c, LSM303_AccelScale scale);
void LSM303_Process_DMA(LSM303* dev);
void LSM303_XferCpltCallback(LSM303* dev, bool is_rx);

bool LSM303_Init(LSM303* dev, I2C_HandleTypeDef* hi2c, LSM303_AccelScale scale);
bool LSM303_StartAccelRead_DMA(LSM303* dev, uint8_t* dma_buffer);
bool LSM303_StartMagRead_DMA(LSM303* dev, uint8_t* dma_buffer);
//bool LSM303_ReadAccel(LSM303* dev, LSM303_Raw* out);
//bool LSM303_ReadMag(LSM303* dev, LSM303_Raw* out);
bool LSM303_ReadTemp(LSM303* dev, int16_t* out);
bool LSM303_ReadTempC(LSM303* dev, float* out_c);

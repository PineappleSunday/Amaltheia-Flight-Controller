#pragma once
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_spi.h"
#include <stdint.h>
#include <stdbool.h>

typedef struct {
    int16_t gx, gy, gz;
} I3GD20_Raw;

typedef struct {
    SPI_HandleTypeDef* hspi; /* SPI handle for communication */
    float dps_per_lsb; /* filled on init */
    int16_t gx_offset; /* zero-rate offset for gx */
    int16_t gy_offset; /* zero-rate offset for gy */
    int16_t gz_offset; /* zero-rate offset for gz */
    bool initialized;  /* flag set on successful init */
} I3GD20;

/* Initialize the I3GD20 device with the provided SPI handle */
bool I3GD20_Init(I3GD20* dev, SPI_HandleTypeDef* hspi);
bool I3GD20_ReadGyro(I3GD20* dev, I3GD20_Raw* out);
/* Calibrate the gyroscope by calculating zero-rate offsets. The device should be stationary. */
void I3GD20_CalibrateZeroRate(I3GD20* dev, uint16_t samples);
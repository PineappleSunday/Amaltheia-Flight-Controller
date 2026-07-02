#pragma once

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

/*
 * BME280 pressure / temperature / humidity driver for STM32 HAL I2C.
 *
 * Compensation formulas and register layout are adapted from Bosch Sensortec's
 * BME280 SensorAPI, BSD-3-Clause, version v3.5.1:
 * https://github.com/boschsensortec/BME280_SensorAPI
 */

#define BME280_I2C_ADDR_PRIM      (0x76u)
#define BME280_I2C_ADDR_SEC       (0x77u)
#define BME280_CHIP_ID            (0x60u)
#define BME280_DEFAULT_TIMEOUT_MS (25u)

typedef enum {
    BME280_OVERSAMPLING_SKIP = 0u,
    BME280_OVERSAMPLING_1X   = 1u,
    BME280_OVERSAMPLING_2X   = 2u,
    BME280_OVERSAMPLING_4X   = 3u,
    BME280_OVERSAMPLING_8X   = 4u,
    BME280_OVERSAMPLING_16X  = 5u
} BME280_Oversampling;

typedef enum {
    BME280_MODE_SLEEP  = 0u,
    BME280_MODE_FORCED = 1u,
    BME280_MODE_NORMAL = 3u
} BME280_Mode;

typedef enum {
    BME280_FILTER_OFF = 0u,
    BME280_FILTER_2   = 1u,
    BME280_FILTER_4   = 2u,
    BME280_FILTER_8   = 3u,
    BME280_FILTER_16  = 4u
} BME280_Filter;

typedef enum {
    BME280_STANDBY_0_5_MS  = 0u,
    BME280_STANDBY_62_5_MS = 1u,
    BME280_STANDBY_125_MS  = 2u,
    BME280_STANDBY_250_MS  = 3u,
    BME280_STANDBY_500_MS  = 4u,
    BME280_STANDBY_1000_MS = 5u,
    BME280_STANDBY_10_MS   = 6u,
    BME280_STANDBY_20_MS   = 7u
} BME280_StandbyTime;

typedef struct {
    BME280_Oversampling osr_temperature;
    BME280_Oversampling osr_pressure;
    BME280_Oversampling osr_humidity;
    BME280_Filter filter;
    BME280_StandbyTime standby_time;
    BME280_Mode mode;
} BME280_Config;

typedef struct {
    uint32_t raw_pressure;
    uint32_t raw_temperature;
    uint32_t raw_humidity;
} BME280_RawData;

typedef struct {
    float temperature_c;
    float pressure_pa;
    float humidity_rh;
    float altitude_m;
} BME280_Data;

typedef struct {
    uint16_t dig_t1;
    int16_t dig_t2;
    int16_t dig_t3;
    uint16_t dig_p1;
    int16_t dig_p2;
    int16_t dig_p3;
    int16_t dig_p4;
    int16_t dig_p5;
    int16_t dig_p6;
    int16_t dig_p7;
    int16_t dig_p8;
    int16_t dig_p9;
    uint8_t dig_h1;
    int16_t dig_h2;
    uint8_t dig_h3;
    int16_t dig_h4;
    int16_t dig_h5;
    int8_t dig_h6;
    int32_t t_fine;
} BME280_CalibData;

typedef struct {
    I2C_HandleTypeDef* hi2c;
    uint8_t addr7;
    uint8_t chip_id;
    uint32_t timeout_ms;
    bool initialized;
    BME280_Config config;
    BME280_CalibData calib;
} BME280;

void BME280_DefaultConfig(BME280_Config* config);
HAL_StatusTypeDef BME280_Probe(I2C_HandleTypeDef* hi2c, uint8_t addr7, uint8_t* chip_id);
bool BME280_Init(BME280* dev, I2C_HandleTypeDef* hi2c, uint8_t addr7);
HAL_StatusTypeDef BME280_SoftReset(BME280* dev);
HAL_StatusTypeDef BME280_SetConfig(BME280* dev, const BME280_Config* config);
HAL_StatusTypeDef BME280_SetMode(BME280* dev, BME280_Mode mode);
HAL_StatusTypeDef BME280_ReadRaw(BME280* dev, BME280_RawData* raw);
HAL_StatusTypeDef BME280_Read(BME280* dev, BME280_Data* data);
HAL_StatusTypeDef BME280_ReadForced(BME280* dev, BME280_Data* data);
uint32_t BME280_GetMeasurementDelayUs(const BME280_Config* config);
float BME280_CalcAltitudeMeters(float pressure_pa, float sea_level_pressure_pa);

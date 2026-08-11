#include "bme280.h"
#include <math.h>
#include <string.h>

#define BME280_REG_CHIP_ID              (0xD0u)
#define BME280_REG_RESET                (0xE0u)
#define BME280_REG_TEMP_PRESS_CALIB     (0x88u)
#define BME280_REG_HUMIDITY_CALIB_H1    (0xA1u)
#define BME280_REG_HUMIDITY_CALIB       (0xE1u)
#define BME280_REG_CTRL_HUM             (0xF2u)
#define BME280_REG_STATUS               (0xF3u)
#define BME280_REG_CTRL_MEAS            (0xF4u)
#define BME280_REG_CONFIG               (0xF5u)
#define BME280_REG_DATA                 (0xF7u)

#define BME280_SOFT_RESET_CMD           (0xB6u)
#define BME280_STATUS_IM_UPDATE         (0x01u)
#define BME280_STATUS_MEASURING         (0x08u)
#define BME280_LEN_TEMP_PRESS_CALIB     (26u)
#define BME280_LEN_HUMIDITY_CALIB       (7u)
#define BME280_LEN_DATA                 (8u)
#define BME280_STANDARD_PRESSURE_PA     (101325.0f)

static uint16_t u16_le(const uint8_t* p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static int16_t s16_le(const uint8_t* p)
{
    return (int16_t)u16_le(p);
}

static int16_t sign_extend_12(uint16_t value)
{
    if ((value & 0x0800u) != 0u) {
        value |= 0xF000u;
    }

    return (int16_t)value;
}

static uint8_t osr_to_multiplier(BME280_Oversampling osr)
{
    switch (osr) {
    case BME280_OVERSAMPLING_1X:  return 1u;
    case BME280_OVERSAMPLING_2X:  return 2u;
    case BME280_OVERSAMPLING_4X:  return 4u;
    case BME280_OVERSAMPLING_8X:  return 8u;
    case BME280_OVERSAMPLING_16X: return 16u;
    case BME280_OVERSAMPLING_SKIP:
    default:                      return 0u;
    }
}

static HAL_StatusTypeDef bme280_read_regs(BME280* dev, uint8_t reg, uint8_t* data, uint16_t len)
{
    if ((dev == NULL) || (dev->hi2c == NULL) || (data == NULL) || (len == 0u)) {
        return HAL_ERROR;
    }

    return HAL_I2C_Mem_Read(dev->hi2c, (uint16_t)(dev->addr7 << 1), reg,
                            I2C_MEMADD_SIZE_8BIT, data, len, dev->timeout_ms);
}

static HAL_StatusTypeDef bme280_write_reg(BME280* dev, uint8_t reg, uint8_t value)
{
    if ((dev == NULL) || (dev->hi2c == NULL)) {
        return HAL_ERROR;
    }

    return HAL_I2C_Mem_Write(dev->hi2c, (uint16_t)(dev->addr7 << 1), reg,
                             I2C_MEMADD_SIZE_8BIT, &value, 1u, dev->timeout_ms);
}

static HAL_StatusTypeDef wait_while_status_set(BME280* dev, uint8_t mask, uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();
    uint8_t status = 0u;

    do {
        if (bme280_read_regs(dev, BME280_REG_STATUS, &status, 1u) != HAL_OK) {
            return HAL_ERROR;
        }

        if ((status & mask) == 0u) {
            return HAL_OK;
        }
    } while ((HAL_GetTick() - start) <= timeout_ms);

    return HAL_TIMEOUT;
}

static void parse_calibration(BME280* dev, const uint8_t tp[BME280_LEN_TEMP_PRESS_CALIB],
                              const uint8_t h[BME280_LEN_HUMIDITY_CALIB])
{
    BME280_CalibData* c = &dev->calib;

    c->dig_t1 = u16_le(&tp[0]);
    c->dig_t2 = s16_le(&tp[2]);
    c->dig_t3 = s16_le(&tp[4]);
    c->dig_p1 = u16_le(&tp[6]);
    c->dig_p2 = s16_le(&tp[8]);
    c->dig_p3 = s16_le(&tp[10]);
    c->dig_p4 = s16_le(&tp[12]);
    c->dig_p5 = s16_le(&tp[14]);
    c->dig_p6 = s16_le(&tp[16]);
    c->dig_p7 = s16_le(&tp[18]);
    c->dig_p8 = s16_le(&tp[20]);
    c->dig_p9 = s16_le(&tp[22]);
    c->dig_h1 = tp[25];
    c->dig_h2 = s16_le(&h[0]);
    c->dig_h3 = h[2];
    c->dig_h4 = sign_extend_12((uint16_t)(((uint16_t)h[3] << 4) | (h[4] & 0x0Fu)));
    c->dig_h5 = sign_extend_12((uint16_t)(((uint16_t)h[5] << 4) | ((h[4] >> 4) & 0x0Fu)));
    c->dig_h6 = (int8_t)h[6];
}

static HAL_StatusTypeDef read_calibration(BME280* dev)
{
    uint8_t tp[BME280_LEN_TEMP_PRESS_CALIB];
    uint8_t h[BME280_LEN_HUMIDITY_CALIB];

    if (bme280_read_regs(dev, BME280_REG_TEMP_PRESS_CALIB, tp, sizeof(tp)) != HAL_OK) {
        return HAL_ERROR;
    }

    if (bme280_read_regs(dev, BME280_REG_HUMIDITY_CALIB, h, sizeof(h)) != HAL_OK) {
        return HAL_ERROR;
    }

    parse_calibration(dev, tp, h);

    if (dev->calib.dig_p1 == 0u) {
        return HAL_ERROR;
    }

    return HAL_OK;
}

static float compensate_temperature(BME280* dev, uint32_t adc_t)
{
    BME280_CalibData* c = &dev->calib;
    int32_t var1;
    int32_t var2;
    int32_t temp_x100;

    var1 = (int32_t)(((((int32_t)(adc_t >> 3)) - ((int32_t)c->dig_t1 << 1)) *
                      ((int32_t)c->dig_t2)) >> 11);
    var2 = (int32_t)(((((((int32_t)(adc_t >> 4)) - ((int32_t)c->dig_t1)) *
                        (((int32_t)(adc_t >> 4)) - ((int32_t)c->dig_t1))) >> 12) *
                      ((int32_t)c->dig_t3)) >> 14);

    c->t_fine = var1 + var2;
    temp_x100 = (int32_t)((c->t_fine * 5 + 128) >> 8);

    return ((float)temp_x100) / 100.0f;
}

static float compensate_pressure(BME280* dev, uint32_t adc_p)
{
    const BME280_CalibData* c = &dev->calib;
    int64_t var1;
    int64_t var2;
    int64_t p;

    var1 = ((int64_t)c->t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)c->dig_p6;
    var2 = var2 + ((var1 * (int64_t)c->dig_p5) << 17);
    var2 = var2 + (((int64_t)c->dig_p4) << 35);
    var1 = ((var1 * var1 * (int64_t)c->dig_p3) >> 8) + ((var1 * (int64_t)c->dig_p2) << 12);
    var1 = (((((int64_t)1) << 47) + var1) * (int64_t)c->dig_p1) >> 33;

    if (var1 == 0) {
        return 0.0f;
    }

    p = 1048576 - (int64_t)adc_p;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)c->dig_p9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)c->dig_p8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)c->dig_p7) << 4);

    return ((float)p) / 256.0f;
}

static float compensate_humidity(BME280* dev, uint32_t adc_h)
{
    const BME280_CalibData* c = &dev->calib;
    int32_t v;

    v = c->t_fine - 76800;
    v = (int32_t)((((((int32_t)adc_h << 14) - (((int32_t)c->dig_h4) << 20) -
                     (((int32_t)c->dig_h5) * v)) + 16384) >> 15) *
                  (((((((v * ((int32_t)c->dig_h6)) >> 10) *
                       (((v * ((int32_t)c->dig_h3)) >> 11) + 32768)) >> 10) +
                     2097152) * ((int32_t)c->dig_h2) + 8192) >> 14));
    v = v - (int32_t)((((v >> 15) * (v >> 15)) >> 7) * ((int32_t)c->dig_h1) >> 4);

    if (v < 0) {
        v = 0;
    } else if (v > 419430400) {
        v = 419430400;
    }

    return ((float)(v >> 12)) / 1024.0f;
}

void BME280_DefaultConfig(BME280_Config* config)
{
    if (config == NULL) return;

    config->osr_temperature = BME280_OVERSAMPLING_2X;
    config->osr_pressure = BME280_OVERSAMPLING_16X;
    config->osr_humidity = BME280_OVERSAMPLING_1X;
    config->filter = BME280_FILTER_16;
    config->standby_time = BME280_STANDBY_62_5_MS;
    config->mode = BME280_MODE_NORMAL;
}

HAL_StatusTypeDef BME280_Probe(I2C_HandleTypeDef* hi2c, uint8_t addr7, uint8_t* chip_id)
{
    HAL_StatusTypeDef status;

    if ((hi2c == NULL) || (chip_id == NULL)) return HAL_ERROR;
    if ((addr7 != BME280_I2C_ADDR_PRIM) && (addr7 != BME280_I2C_ADDR_SEC)) return HAL_ERROR;

    *chip_id = 0u;

    status = HAL_I2C_IsDeviceReady(hi2c, (uint16_t)(addr7 << 1), 1u, 10u);
    if (status != HAL_OK) return status;

    return HAL_I2C_Mem_Read(hi2c, (uint16_t)(addr7 << 1), BME280_REG_CHIP_ID,
                            I2C_MEMADD_SIZE_8BIT, chip_id, 1u, 10u);
}

bool BME280_Init(BME280* dev, I2C_HandleTypeDef* hi2c, uint8_t addr7)
{
    BME280_Config config;
    uint8_t chip_id = 0u;

    if ((dev == NULL) || (hi2c == NULL)) return false;
    if ((addr7 != BME280_I2C_ADDR_PRIM) && (addr7 != BME280_I2C_ADDR_SEC)) return false;

    memset(dev, 0, sizeof(*dev));
    dev->hi2c = hi2c;
    dev->addr7 = addr7;
    dev->timeout_ms = BME280_DEFAULT_TIMEOUT_MS;

    if (bme280_read_regs(dev, BME280_REG_CHIP_ID, &chip_id, 1u) != HAL_OK) return false;
    if (chip_id != BME280_CHIP_ID) return false;

    dev->chip_id = chip_id;

    if (BME280_SoftReset(dev) != HAL_OK) return false;
    if (read_calibration(dev) != HAL_OK) return false;

    BME280_DefaultConfig(&config);
    if (BME280_SetConfig(dev, &config) != HAL_OK) return false;

    dev->initialized = true;
    return true;
}

HAL_StatusTypeDef BME280_SoftReset(BME280* dev)
{
    HAL_StatusTypeDef status;

    status = bme280_write_reg(dev, BME280_REG_RESET, BME280_SOFT_RESET_CMD);
    if (status != HAL_OK) return status;

    HAL_Delay(2u);
    return wait_while_status_set(dev, BME280_STATUS_IM_UPDATE, 20u);
}

HAL_StatusTypeDef BME280_SetConfig(BME280* dev, const BME280_Config* config)
{
    HAL_StatusTypeDef status;
    uint8_t ctrl_hum;
    uint8_t ctrl_meas;
    uint8_t cfg;

    if ((dev == NULL) || (config == NULL)) return HAL_ERROR;

    status = BME280_SetMode(dev, BME280_MODE_SLEEP);
    if (status != HAL_OK) return status;

    ctrl_hum = (uint8_t)(config->osr_humidity & 0x07u);
    cfg = (uint8_t)(((uint8_t)(config->standby_time & 0x07u) << 5) |
                    ((uint8_t)(config->filter & 0x07u) << 2));
    ctrl_meas = (uint8_t)(((uint8_t)(config->osr_temperature & 0x07u) << 5) |
                          ((uint8_t)(config->osr_pressure & 0x07u) << 2) |
                          ((uint8_t)(config->mode & 0x03u)));

    status = bme280_write_reg(dev, BME280_REG_CTRL_HUM, ctrl_hum);
    if (status != HAL_OK) return status;

    status = bme280_write_reg(dev, BME280_REG_CONFIG, cfg);
    if (status != HAL_OK) return status;

    status = bme280_write_reg(dev, BME280_REG_CTRL_MEAS, ctrl_meas);
    if (status != HAL_OK) return status;

    dev->config = *config;
    return HAL_OK;
}

HAL_StatusTypeDef BME280_SetMode(BME280* dev, BME280_Mode mode)
{
    uint8_t ctrl_meas;
    HAL_StatusTypeDef status;

    if (dev == NULL) return HAL_ERROR;

    status = bme280_read_regs(dev, BME280_REG_CTRL_MEAS, &ctrl_meas, 1u);
    if (status != HAL_OK) return status;

    ctrl_meas = (uint8_t)((ctrl_meas & ~0x03u) | ((uint8_t)mode & 0x03u));
    status = bme280_write_reg(dev, BME280_REG_CTRL_MEAS, ctrl_meas);
    if (status == HAL_OK) {
        dev->config.mode = mode;
    }

    return status;
}

HAL_StatusTypeDef BME280_ReadRaw(BME280* dev, BME280_RawData* raw)
{
    uint8_t data[BME280_LEN_DATA];

    if ((dev == NULL) || (raw == NULL)) return HAL_ERROR;

    if (bme280_read_regs(dev, BME280_REG_DATA, data, sizeof(data)) != HAL_OK) {
        return HAL_ERROR;
    }

    raw->raw_pressure = (((uint32_t)data[0] << 12) | ((uint32_t)data[1] << 4) | ((uint32_t)data[2] >> 4));
    raw->raw_temperature = (((uint32_t)data[3] << 12) | ((uint32_t)data[4] << 4) | ((uint32_t)data[5] >> 4));
    raw->raw_humidity = (((uint32_t)data[6] << 8) | (uint32_t)data[7]);

    return HAL_OK;
}

HAL_StatusTypeDef BME280_Read(BME280* dev, BME280_Data* data)
{
    BME280_RawData raw;

    if ((dev == NULL) || (data == NULL)) return HAL_ERROR;
    if (BME280_ReadRaw(dev, &raw) != HAL_OK) return HAL_ERROR;

    data->temperature_c = compensate_temperature(dev, raw.raw_temperature);
    data->pressure_pa = compensate_pressure(dev, raw.raw_pressure);
    data->humidity_rh = compensate_humidity(dev, raw.raw_humidity);
    data->altitude_m = BME280_CalcAltitudeMeters(data->pressure_pa, BME280_STANDARD_PRESSURE_PA);

    return HAL_OK;
}

HAL_StatusTypeDef BME280_ReadForced(BME280* dev, BME280_Data* data)
{
    HAL_StatusTypeDef status;

    if ((dev == NULL) || (data == NULL)) return HAL_ERROR;

    status = BME280_SetMode(dev, BME280_MODE_FORCED);
    if (status != HAL_OK) return status;

    HAL_Delay((BME280_GetMeasurementDelayUs(&dev->config) + 999u) / 1000u);

    status = wait_while_status_set(dev, BME280_STATUS_MEASURING, 100u);
    if (status != HAL_OK) return status;

    return BME280_Read(dev, data);
}

uint32_t BME280_GetMeasurementDelayUs(const BME280_Config* config)
{
    uint32_t delay_us = 1250u;

    if (config == NULL) return 10000u;

    delay_us += 2300u * osr_to_multiplier(config->osr_temperature);

    if (config->osr_pressure != BME280_OVERSAMPLING_SKIP) {
        delay_us += (2300u * osr_to_multiplier(config->osr_pressure)) + 575u;
    }

    if (config->osr_humidity != BME280_OVERSAMPLING_SKIP) {
        delay_us += (2300u * osr_to_multiplier(config->osr_humidity)) + 575u;
    }

    return delay_us;
}

float BME280_CalcAltitudeMeters(float pressure_pa, float sea_level_pressure_pa)
{
    if ((pressure_pa <= 0.0f) || (sea_level_pressure_pa <= 0.0f)) {
        return 0.0f;
    }

    return 44330.0f * (1.0f - powf(pressure_pa / sea_level_pressure_pa, 0.19029495f));
}

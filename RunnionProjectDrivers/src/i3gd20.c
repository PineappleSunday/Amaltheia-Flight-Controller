// Arbiter Industries Inc. 
// The arbiter of defensive technologies
// Author: C.RUNNION
// GYROSCOPE I3GD20 DRIVER
// I3GD20 & I3G4200D driver using SPI HAL interface
// References: 
// I3GD20 datasheet: https://www.st.com/resource/en/datasheet/I3gd20.pdf
// I3G4200D datasheet: https://www.st.com/resource/en/datasheet/i3g4200d.pdf



#include "i3gd20.h"
#include "main.h" // for CS pin definitions
#include <string.h>
#include <stdio.h>

/* I3GD20 registers */
#define I3GD20_WHO_AM_I      0x0F
#define I3GD20_CTRL_REG1     0x20
#define I3GD20_CTRL_REG4     0x23
#define I3GD20_OUT_X_L       0x28
#define I3GD20_STATUS_REG    0x27
#define I3GD20_STATUS_ZYXDA  0x08  // XYZ data available

#define I3GD20_WHO_AM_I_VALUE1 0xD4
#define I3GD20_WHO_AM_I_VALUE2 0xD7

/* I3G4200D WHO_AM_I */
#define I3G4200D_WHO_AM_I 0xD3

/* I3G4250D WHO_AM_I */
#define I3G4250D_WHO_AM_I 0xD8

/* Helper: CS control */
static inline void i3gd20_cs_assert(void) {
    HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);
}
static inline void i3gd20_cs_deassert(void) {
    HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_SET);
}

/* Helper: Write a register */
static bool i3gd20_write_reg(SPI_HandleTypeDef* hspi, uint8_t reg, uint8_t val)
{
    uint8_t tx[2] = { reg, val };
    printf("I3GD20 Write: reg 0x%02X = 0x%02X\r\n", reg, val);

    i3gd20_cs_assert();
    HAL_StatusTypeDef status = HAL_SPI_Transmit(hspi, tx, 2, 50);
    i3gd20_cs_deassert(); // De-assert CS immediately after transaction

    if (status != HAL_OK) {
        printf("I3GD20 Write failed: HAL_Status = %d\r\n", status);
        return false;
    }
    return true;
}

/* Helper: Read register(s) */
static bool i3gd20_read_reg(SPI_HandleTypeDef* hspi, uint8_t reg, uint8_t *buf, uint16_t len)
{
    if (len == 0) return false;
    uint8_t addr = reg | 0x80; // read
    if (len > 1) addr |= 0x40; // auto-increment
    
    uint8_t rx[8]; // max buffer for our needs
    uint8_t tx[8];
    if (len > sizeof(rx)-1) return false; // safety check
    
    // Setup transmit buffer with address and dummy bytes
    tx[0] = addr;
    for (int i = 1; i <= len; i++) tx[i] = 0xFF; // dummy bytes for read
    
    i3gd20_cs_assert();
    // Single transaction with dummy bytes for read
    if (HAL_SPI_TransmitReceive(hspi, tx, rx, len + 1, 100) != HAL_OK) {
        i3gd20_cs_deassert();
        return false;
    }
    i3gd20_cs_deassert();
    
    // Copy received data (skip first byte which was during address transmission)
    memcpy(buf, rx + 1, len);
    return true;
}

bool I3GD20_Init(I3GD20* dev, SPI_HandleTypeDef* hspi)
{
    if (dev == NULL || hspi == NULL) return false;
    dev->hspi = hspi;
    dev->initialized = false;
    dev->dps_per_lsb = 0.00875f;

    // 1. Ensure CS is High (Inactive)
    i3gd20_cs_deassert();
    HAL_Delay(100); // Wait for power-up

    // 2. Dummy Read to clear SPI bus
    // Sometimes the first transaction is garbage after reset
    uint8_t dummy;
    i3gd20_read_reg(dev->hspi, I3GD20_WHO_AM_I, &dummy, 1);
    HAL_Delay(10);

    // 3. Attempt to read WHO_AM_I multiple times
    // This handles cases where the sensor needs a few clock cycles to wake up
    uint8_t who = 0;
    bool found = false;

    for (int i = 0; i < 5; i++) {
        if (i3gd20_read_reg(dev->hspi, I3GD20_WHO_AM_I, &who, 1)) {
            if (who == I3GD20_WHO_AM_I_VALUE1 || who == I3GD20_WHO_AM_I_VALUE2 ||
                who == I3G4200D_WHO_AM_I || who == I3G4250D_WHO_AM_I) {
                found = true;
                break;
            }
        }
        HAL_Delay(10);
    }

    if (!found) {
        printf("Gyro Init Failed. Last WHO_AM_I = 0x%02X\r\n", who);
        return false;
    }

    printf("Gyro Detected! ID = 0x%02X\r\n", who);

    // 4. Configure Control Registers
    // CTRL1: 0x0F (Normal Mode, XYZ enabled, 100Hz)
    // CTRL4: 0x80 (Block Data Update ON, 250dps) -> Safer for reading
    if (!i3gd20_write_reg(dev->hspi, I3GD20_CTRL_REG1, 0x0F)) return false;
    if (!i3gd20_write_reg(dev->hspi, I3GD20_CTRL_REG4, 0x80)) return false; // Changed to 0x80 (BDU)

    dev->initialized = true;
    return true;
}

void I3GD20_CalibrateZeroRate(I3GD20* dev, uint16_t samples)
{
    // Validating initialization of gyrscope
    if (!dev || !dev->hspi || !dev->initialized) return;
    
    printf("Beginning Gyroscrope Zero-Rate Calibration with %d samples...\r\n", samples);

    int32_t sum_x = 0, sum_y = 0, sum_z = 0;
    I3GD20_Raw sample;
    for (uint16_t i = 0; i < samples; i++) {
        if (I3GD20_ReadGyro(dev, &sample)) {
            sum_x += sample.gx;
            sum_y += sample.gy;
            sum_z += sample.gz;
        }
        HAL_Delay(5); // small delay between samples
    }

    // Calulate average offsets
    dev->gx_offset = (int16_t)(sum_x / samples);
    dev->gy_offset = (int16_t)(sum_y / samples);
    dev->gz_offset = (int16_t)(sum_z / samples);
    
    // return sum of offsets as simple check
    printf("I3GD20 Calibration offsets: gx=%d, gy=%d, gz=%d\r\n", dev->gx_offset, dev->gy_offset, dev->gz_offset);
}

bool I3GD20_ReadGyro(I3GD20* dev, I3GD20_Raw* out)
{
    if (!dev || !out || !dev->hspi) return false;
    
    // Always read the output registers (skip relying on STATUS for now)
    uint8_t buf[6];
    if (!i3gd20_read_reg(dev->hspi, I3GD20_OUT_X_L, buf, 6)) {
        printf("I3GD20: Failed to read OUT_X..OUT_Z\r\n");
        return false;
    }
    
    // DEBUG: Print raw values occasionally so we can see if bytes change
    /*
    static uint32_t last_debug = 0;
    if (HAL_GetTick() - last_debug > 1000) {
        printf("I3GD20 raw: %02X %02X  %02X %02X  %02X %02X\r\n",
               buf[0], buf[1], buf[2], buf[3], buf[4], buf[5]);
        last_debug = HAL_GetTick();
    } */
    
    // Combine bytes and apply offsets
    out->gx = (int16_t)( (int16_t)buf[1] << 8 | buf[0] ) - dev->gx_offset;
    out->gy = (int16_t)( (int16_t)buf[3] << 8 | buf[2] ) - dev->gy_offset;
    out->gz = (int16_t)( (int16_t)buf[5] << 8 | buf[4] ) - dev->gz_offset;
    return true;
}

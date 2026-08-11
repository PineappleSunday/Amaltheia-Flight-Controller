// Arbiter Industries Inc. 
// The arbiter of defensive technologies
// Author: C.RUNNION
// ACCELEROMETER & MAGNETOMETER LSM303 DRIVER
// LSM303 driver supporting both DLHC and AGR variants
// Supports accelerometer and magnetometer reading (and temperature for AGR)
// Uses I2C HAL interface
// References:
// LSM303DLHC datasheet: https://www.st.com/resource/en/datasheet/lsm303dlhc.pdf
// LSM303AGR datasheet: https://www.st.com/resource/en/datasheet/


#include "lsm303.h"
#include "calibration.h"
#include <string.h>
#include <stdio.h>

#define I2C_TIMEOUT 50

// --- DLHC registers (Accel) ---
#define DLHC_WHO_AM_I_A     0x0F   // expect 0x33
#define DLHC_CTRL1_A        0x20
#define DLHC_CTRL4_A        0x23
#define DLHC_OUT_X_L_A      0x28   // auto-increment with MS bit

// --- DLHC registers (Mag) ---
#define DLHC_CRA_M          0x00
#define DLHC_CRB_M          0x01
#define DLHC_MR_M           0x02
#define DLHC_OUT_X_H_M      0x03   // XH, XL, ZH, ZL, YH, YL (note order)
#define DLHC_SR_M           0x09
// ID A,B,C at 0x0A..0x0C -> 'H', '4', '3'

// --- AGR registers (Accel) ---
#define AGR_WHO_AM_I_A      0x0F   // expect 0x33
#define AGR_CTRL1_A         0x20
#define AGR_CTRL2_A         0x21
#define AGR_CTRL3_A         0x22
#define AGR_CTRL4_A         0x23
#define AGR_CTRL5_A         0x24
#define AGR_CTRL6_A         0x25
#define AGR_REF_DATA_A      0x26
#define AGR_STATUS_A        0x27
#define AGR_OUT_X_L_A       0x28

// --- AGR registers (Mag) ---
#define AGR_WHO_AM_I_M      0x4F   // expect 0x40
#define AGR_CFG_A_M         0x60
#define AGR_CFG_B_M         0x61
#define AGR_CFG_C_M         0x62
#define AGR_OUTX_L_M        0x68   // OUTX_L..OUTZ_H continuous
#define AGR_STATUS_M        0x67
#define AGR_CFG_A_M_REBOOT  0x40
#define AGR_CFG_A_M_SOFT_RST 0x20


// --- AGR Registers (Temp) ---
#define AGR_TEMP_CFG_A     0x1F
#define AGR_OUT_TEMP_L_A   0x0C
#define AGR_OUT_TEMP_H_A   0x0D
#define LSM303AGR_TEMPSENSOR_ENABLE         ((uint8_t) 0x80)   /*!< Temp sensor Enable */
#define LSM303AGR_TEMPSENSOR_DISABLE        ((uint8_t) 0x00)   /*!< Temp sensor Disable */

// --- KEEP THESE AT THE TOP ---
static HAL_StatusTypeDef i2c_write(I2C_HandleTypeDef* hi2c, uint8_t addr7, uint8_t reg, uint8_t val) {
	return HAL_I2C_Mem_Write(hi2c, addr7<<1, reg, I2C_MEMADD_SIZE_8BIT, &val, 1, I2C_TIMEOUT);
}

static HAL_StatusTypeDef i2c_read(I2C_HandleTypeDef* hi2c, uint8_t addr7, uint8_t reg, uint8_t* buf, uint16_t len) {
	return HAL_I2C_Mem_Read(hi2c, addr7<<1, reg, I2C_MEMADD_SIZE_8BIT, buf, len, I2C_TIMEOUT);
}

static uint8_t rd8(I2C_HandleTypeDef* hi2c, uint8_t a, uint8_t r) {
	uint8_t v=0; i2c_read(hi2c, a, r, &v, 1); return v;
}

static int16_t little_endian_i16(uint8_t lo, uint8_t hi) {
	return (int16_t)((uint16_t)lo | ((uint16_t)hi << 8));
}

// Internal helper to kick off the next phase
void LSM303_Process_DMA(LSM303* dev) {
	if (dev->state != LSM303_IDLE) return;

	// Start with Accel
	uint8_t reg = DLHC_OUT_X_L_A | 0x80;
	dev->state = LSM303_ACCEL_TX;
	HAL_I2C_Master_Transmit_DMA(dev->hi2c, dev->addr_acc << 1, &reg, 1);
}

// Logic to handle the transition between TX (addressing) and RX (reading)
void LSM303_XferCpltCallback(LSM303* dev, bool is_rx) {
	if (!is_rx) { // TX Complete
		if (dev->state == LSM303_ACCEL_TX) {
			dev->state = LSM303_ACCEL_RX;
			HAL_I2C_Master_Receive_DMA(dev->hi2c, dev->addr_acc << 1, dev->accel_raw, 6);
		} else if (dev->state == LSM303_MAG_TX) {
			dev->state = LSM303_MAG_RX;
			// Just start the receive; the address was already set in the TX phase
			HAL_I2C_Master_Receive_DMA(dev->hi2c, dev->addr_mag << 1, dev->mag_raw, 6);
		}
	} else { // RX Complete (Data Received)
		if (dev->state == LSM303_ACCEL_RX) {
			dev->accel_ready = true;

			// --- THE FIX: CHAIN TO MAG ---
			dev->state = LSM303_MAG_TX;
			uint8_t reg = (dev->variant == LSM303_DLHC) ? DLHC_OUT_X_H_M : AGR_OUTX_L_M;
			if (HAL_I2C_Master_Transmit_DMA(dev->hi2c, dev->addr_mag << 1, &reg, 1) != HAL_OK) {
				dev->state = LSM303_IDLE; // Reset if hardware fails
			}
		}
		else if (dev->state == LSM303_MAG_RX) {
			dev->mag_ready = true;
			dev->state = LSM303_IDLE; // Finally done with the whole sequence
		}
	}
}
// Try reading both maps to decide variant
static LSM303_Variant probe_variant(LSM303* dev) {
	// First, accel WHO_AM_I (both should be 0x33)
	uint8_t who_a = rd8(dev->hi2c, dev->addr_acc, DLHC_WHO_AM_I_A);
	dev->accel_who_am_i = who_a;
	printf("LSM303: accel WHO_AM_I = 0x%02X\r\n", who_a);
	if (who_a != 0x33) return LSM303_UNKNOWN;

	// Try AGR magnetometer WHO_AM_I (0x4F -> 0x40)
	uint8_t who_m = rd8(dev->hi2c, dev->addr_mag, AGR_WHO_AM_I_M);
	dev->mag_who_am_i_agr = who_m;
	printf("LSM303: mag WHO_AM_I probe = 0x%02X\r\n", who_m);
	if (who_m == 0x40) return LSM303_AGR;

	// DLHC does not have WHO_AM_I_M; read ID A/B/C at 0x0A..0x0C -> 'H','4','3'
	uint8_t ida = rd8(dev->hi2c, dev->addr_mag, 0x0A);
	uint8_t idb = rd8(dev->hi2c, dev->addr_mag, 0x0B);
	uint8_t idc = rd8(dev->hi2c, dev->addr_mag, 0x0C);
	dev->mag_id_a = ida;
	dev->mag_id_b = idb;
	dev->mag_id_c = idc;
	printf("LSM303: mag ID bytes = 0x%02X 0x%02X 0x%02X ('%c' '%c' '%c')\r\n", ida, idb, idc, ida, idb, idc);
	if (ida=='H' && idb=='4' && idc=='3') return LSM303_DLHC;

	return LSM303_UNKNOWN;
}

bool LSM303_Init(LSM303* dev, I2C_HandleTypeDef* hi2c, LSM303_AccelScale scale) {
	dev->hi2c = hi2c;
	dev->addr_acc = 0x19; // SA0=1 default on many boards
	dev->addr_mag = 0x1E; // default
	dev->variant  = LSM303_UNKNOWN;
	dev->accel_who_am_i = 0u;
	dev->mag_who_am_i_agr = 0u;
	dev->mag_id_a = 0u;
	dev->mag_id_b = 0u;
	dev->mag_id_c = 0u;
	dev->accel_ctrl1 = 0u;
	dev->accel_ctrl4 = 0u;
	dev->mag_cfg_a = 0u;
	dev->mag_cfg_b = 0u;
	dev->mag_cfg_c = 0u;
	dev->temp_cfg = 0u;

	// --- Accel: 100 Hz ODR, all axes enable, normal mode ---
	// CTRL1_A: ODR=100Hz(0b0101<<4), Xen=Yen=Zen=1 => 0x57
	if (i2c_write(hi2c, dev->addr_acc, DLHC_CTRL1_A, 0x57) != HAL_OK) return false;
	dev->accel_ctrl1 = rd8(hi2c, dev->addr_acc, DLHC_CTRL1_A);

	// --- Accel scale and resolution ---
	// CTRL4_A: BDU=1 (bit 7), HR=1 (bit 3, AGR only), FS (bits 5-4)
	uint8_t ctrl4_val = 0x88; // BDU=1, HR=1
	float lsb_per_g;
	switch(scale) {
	case LSM303_ACCEL_SCALE_2G:  ctrl4_val |= (0b00 << 4); lsb_per_g = 1000.0f; break;
	case LSM303_ACCEL_SCALE_4G:  ctrl4_val |= (0b01 << 4); lsb_per_g = 500.0f;  break;
	case LSM303_ACCEL_SCALE_8G:  ctrl4_val |= (0b10 << 4); lsb_per_g = 250.0f;  break;
	case LSM303_ACCEL_SCALE_16G: ctrl4_val |= (0b11 << 4); lsb_per_g = 83.3333f; break;
	default: return false; // Invalid scale
	}
	if (i2c_write(hi2c, dev->addr_acc, DLHC_CTRL4_A, ctrl4_val) != HAL_OK) return false;
	dev->accel_ctrl4 = rd8(hi2c, dev->addr_acc, DLHC_CTRL4_A);

	// High-resolution accel samples are left-justified 12-bit values.
	// The value here is the number of right-shifted counts per g.
	dev->accel_g_per_lsb = 1.0f / lsb_per_g;
	dev->accel_bias_counts[0] = 0.0f;
	dev->accel_bias_counts[1] = 0.0f;
	dev->accel_bias_counts[2] = 0.0f;
	dev->accel_counts_per_g[0] = lsb_per_g;
	dev->accel_counts_per_g[1] = lsb_per_g;
	dev->accel_counts_per_g[2] = lsb_per_g;

	// --- Magnetometer init: detect variant ---
	dev->variant = probe_variant(dev);
	if (dev->variant == LSM303_UNKNOWN) return false;

	if (dev->variant == LSM303_DLHC) {
		// CRA_REG_M: Data rate 75 Hz (0b110 << 2) -> 0x18
		i2c_write(hi2c, dev->addr_mag, DLHC_CRA_M, 0x18);
		// CRB_REG_M: Gain ±1.3 gauss (0x20), we’ll scale later
		i2c_write(hi2c, dev->addr_mag, DLHC_CRB_M, 0x20);
		// MR_REG_M: Continuous-conversion mode (0x00)
		i2c_write(hi2c, dev->addr_mag, DLHC_MR_M,  0x00);
		dev->mag_cfg_a = rd8(hi2c, dev->addr_mag, DLHC_CRA_M);
		dev->mag_cfg_b = rd8(hi2c, dev->addr_mag, DLHC_CRB_M);
		dev->mag_cfg_c = rd8(hi2c, dev->addr_mag, DLHC_MR_M);
		// Scale: datasheet ~1100 LSB/gauss on X/Y at ±1.3g range; use 1/1100 as a starting point
		dev->mag_gauss_per_lsb = 1.0f / 1100.0f;

	} else { // LSM303_AGR
		for (uint8_t i = 0; i < 3u; i++) {
			dev->accel_bias_counts[i] = g_lsm303agr_accel_cal.bias[i];
			dev->accel_counts_per_g[i] = g_lsm303agr_accel_cal.counts_per_g[i];
		}
		// CFG_REG_A_M: Temperature comp=1, ODR=100Hz (0b100 << 2), LPF=1 => 0b1 100 1 00 = 0x9C
		i2c_write(hi2c, dev->addr_mag, AGR_CFG_A_M, 0x9C);
		// CFG_REG_B_M: OFF_CANC=1 (offset cancel), LPF=1 (already set), set range ±50 gauss fixed on AGR
		i2c_write(hi2c, dev->addr_mag, AGR_CFG_B_M, 0x01);
		// CFG_REG_C_M: Continuous-conversion MD=00
		i2c_write(hi2c, dev->addr_mag, AGR_CFG_C_M, 0x00);
		// enable temp sensor
		i2c_write(hi2c, dev->addr_acc, AGR_TEMP_CFG_A, LSM303AGR_TEMPSENSOR_ENABLE);
		dev->mag_cfg_a = rd8(hi2c, dev->addr_mag, AGR_CFG_A_M);
		dev->mag_cfg_b = rd8(hi2c, dev->addr_mag, AGR_CFG_B_M);
		dev->mag_cfg_c = rd8(hi2c, dev->addr_mag, AGR_CFG_C_M);
		dev->temp_cfg = rd8(hi2c, dev->addr_acc, AGR_TEMP_CFG_A);
		dev->mag_gauss_per_lsb = 0.0015f;
	}
	return true;
}

bool LSM303_SoftResetAGR(LSM303* dev) {
	if (!dev || !dev->hi2c) return false;
	if (dev->variant != LSM303_AGR) return false;

	// Preserve current config bits, pulse SOFT_RST then REBOOT.
	uint8_t cfg_a = rd8(dev->hi2c, dev->addr_mag, AGR_CFG_A_M);
	if (i2c_write(dev->hi2c, dev->addr_mag, AGR_CFG_A_M, (uint8_t)(cfg_a | AGR_CFG_A_M_SOFT_RST)) != HAL_OK) {
		return false;
	}
	HAL_Delay(5);

	cfg_a = rd8(dev->hi2c, dev->addr_mag, AGR_CFG_A_M);
	if (i2c_write(dev->hi2c, dev->addr_mag, AGR_CFG_A_M, (uint8_t)(cfg_a | AGR_CFG_A_M_REBOOT)) != HAL_OK) {
		return false;
	}
	HAL_Delay(5);

	return true;
}

bool LSM303_Recover(LSM303* dev, LSM303_AccelScale scale) {
	if (!dev || !dev->hi2c) return false;

	// AGR supports software reset/reboot; DLHC falls back to full re-init.
	if (dev->variant == LSM303_AGR) {
		(void)LSM303_SoftResetAGR(dev);
	}

	// Reset software state machine flags before re-initializing.
	dev->state = LSM303_IDLE;
	dev->accel_ready = false;
	dev->mag_ready = false;

	return LSM303_Init(dev, dev->hi2c, scale);
}

// Non-blocking Accelerometer Start
bool LSM303_StartAccelRead_DMA(LSM303* dev, uint8_t* dma_buffer) {
	if (!dev || !dev->hi2c || !dma_buffer) return false;

	// Use the auto-increment bit (0x80) to read all 6 bytes (XL to ZH) in one shot
	HAL_StatusTypeDef status = HAL_I2C_Mem_Read_DMA(dev->hi2c,
			(dev->addr_acc << 1),
			DLHC_OUT_X_L_A | 0x80,
			I2C_MEMADD_SIZE_8BIT,
			dma_buffer, 6);
	return (status == HAL_OK);
}

// Non-blocking Magnetometer Start (AGR variant example)
bool LSM303_StartMagRead_DMA(LSM303* dev, uint8_t* dma_buffer) {
	if (!dev || !dev->hi2c || !dma_buffer) return false;

	uint8_t reg_addr = (dev->variant == LSM303_DLHC) ? 0x03 : 0x68;

	// Use HAL_I2C_Mem_Read_DMA to handle the memory-mapped register access
	HAL_StatusTypeDef status = HAL_I2C_Mem_Read_DMA(dev->hi2c,
			(dev->addr_mag << 1),
			reg_addr,
			I2C_MEMADD_SIZE_8BIT,
			dma_buffer, 6);
	return (status == HAL_OK);
}

void LSM303_DecodeAccelG(const LSM303* dev, const uint8_t raw[6], float* ax_g, float* ay_g, float* az_g) {
	if (!dev || !raw || !ax_g || !ay_g || !az_g) return;

	// AGR High Resolution mode: 12-bit left-justified, so shift right by 4 after sign extension.
	int16_t x = (int16_t)(little_endian_i16(raw[0], raw[1]) >> 4);
	int16_t y = (int16_t)(little_endian_i16(raw[2], raw[3]) >> 4);
	int16_t z = (int16_t)(little_endian_i16(raw[4], raw[5]) >> 4);

	*ax_g = ((float)x - dev->accel_bias_counts[0]) / dev->accel_counts_per_g[0];
	*ay_g = ((float)y - dev->accel_bias_counts[1]) / dev->accel_counts_per_g[1];
	*az_g = ((float)z - dev->accel_bias_counts[2]) / dev->accel_counts_per_g[2];
}

bool LSM303_ReadTemp(LSM303* dev, int16_t* out) {
	// Temperature sensor is only on the AGR variant
	if (dev->variant != LSM303_AGR) {
		return false;
	}

	uint8_t buf[2];
	// Set auto-increment bit (MSB of register address) for multi-byte read
	if (HAL_OK != i2c_read(dev->hi2c, dev->addr_acc, AGR_OUT_TEMP_L_A | 0x80, buf, 2)) {
		return false;
	} else {
		// The data is a 12-bit left-justified value, so we can treat it as a 16-bit signed int.
		// Casting to (int16_t) directly performs 2's complement interpretation automatically.
		*out = (int16_t)((buf[1]<<8)|buf[0]);
		return true;
	}
}

bool LSM303_ReadTempC(LSM303* dev, float* out_c) {
	int16_t raw_temp;
	if (!LSM303_ReadTemp(dev, &raw_temp)) {
		return false;
	}

	// From the datasheet AN4602:
	// The data is left-justified. The output is 12 bits.
	// Sensitivity is 256 LSB/°C. A value of 0 corresponds to 25 °C.
	// So, we shift right by 4 to get the 12-bit value.
	*out_c = 25.0f + ((float)(raw_temp >> 4) / 256.0f);
	return true;
}

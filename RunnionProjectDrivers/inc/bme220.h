#pragma once

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

/*
 * BME-220 / TOFSense UART framing (NLink Frame0):
 *  - Header:      0x57
 *  - Function:    0x00 (active output)
 *  - Frame length 16 bytes (including checksum)
 *  - Checksum:    low 8 bits of sum(frame[0..14])
 */

#define BME220_FRAME_HEADER                (0x57u)
#define BME220_FUNC_OUTPUT_FRAME0          (0x00u)
#define BME220_FUNC_QUERY_FRAME0           (0x10u)
#define BME220_FRAME0_LEN_BYTES            (16u)
#define BME220_QUERY_FRAME0_LEN_BYTES      (8u)
#define BME220_DEFAULT_BAUD                (921600u)

typedef struct {
    uint8_t node_id;
    uint32_t system_time_ms;
    uint32_t distance_mm;
    float distance_m;
    uint8_t distance_status;
    uint16_t signal_strength;
    uint8_t reserved0;
    uint8_t reserved1;
    uint32_t rx_tick_ms;
} BME220_Frame0;

typedef struct {
    UART_HandleTypeDef* huart;

    // Optional UART DMA circular-buffer integration.
    uint8_t* dma_rx_buffer;
    uint16_t dma_rx_buffer_len;
    uint16_t dma_read_idx;

    // Streaming parser state.
    uint8_t frame_buf[BME220_FRAME0_LEN_BYTES];
    uint8_t frame_index;
    bool in_sync;
    bool fresh_frame_ready;

    // Diagnostics.
    uint32_t frames_ok;
    uint32_t frames_bad_checksum;
    uint32_t frames_bad_format;

    // Last decoded measurement.
    BME220_Frame0 latest;
} BME220;

bool BME220_Init(BME220* dev, UART_HandleTypeDef* huart);
void BME220_ResetParser(BME220* dev);

// Optional DMA ring attachment/start helpers.
bool BME220_AttachDMARxBuffer(BME220* dev, uint8_t* dma_rx_buffer, uint16_t dma_rx_buffer_len);
HAL_StatusTypeDef BME220_StartRxDMA(BME220* dev);
void BME220_ProcessDMARing(BME220* dev);

// Streaming parse entry points.
void BME220_ProcessByte(BME220* dev, uint8_t byte_in);
void BME220_ProcessBuffer(BME220* dev, const uint8_t* data, uint16_t len);

// Query mode helper (NLink read frame0).
bool BME220_BuildQueryFrame0(uint8_t node_id, uint8_t out_frame[BME220_QUERY_FRAME0_LEN_BYTES]);
HAL_StatusTypeDef BME220_SendQueryFrame0(BME220* dev, uint8_t node_id, uint32_t timeout_ms);

// Raw frame parser for tests or offline parsing.
bool BME220_ParseFrame0(const uint8_t frame[BME220_FRAME0_LEN_BYTES], BME220_Frame0* out);

// Accessors.
bool BME220_GetLatest(const BME220* dev, BME220_Frame0* out);
bool BME220_HasFreshFrame(const BME220* dev);
void BME220_ClearFreshFlag(BME220* dev);

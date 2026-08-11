#pragma once

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

/*
 * GT-U7 GPS driver.
 *
 * Most GT-U7 modules expose a u-blox-compatible UART that streams NMEA 0183
 * sentences. This driver validates NMEA checksums and decodes GGA/RMC data.
 */

#define GT_U7_DEFAULT_BAUD       (9600u)
#define GT_U7_MAX_SENTENCE_LEN   (96u)

typedef struct {
    bool valid;
    uint8_t fix_quality;      // GGA: 0 invalid, 1 GPS, 2 DGPS, etc.
    uint8_t satellites;
    float hdop;

    double latitude_deg;
    double longitude_deg;
    float altitude_m;
    float geoid_sep_m;
    float speed_mps;
    float course_deg;

    uint8_t utc_hours;
    uint8_t utc_minutes;
    uint8_t utc_seconds;
    uint16_t utc_milliseconds;
    uint8_t date_day;
    uint8_t date_month;
    uint16_t date_year;

    uint32_t rx_tick_ms;
} GTU7_Fix;

typedef struct {
    UART_HandleTypeDef* huart;

    uint8_t* dma_rx_buffer;
    uint16_t dma_rx_buffer_len;
    uint16_t dma_read_idx;

    char sentence[GT_U7_MAX_SENTENCE_LEN];
    uint16_t sentence_len;
    bool collecting;

    uint32_t sentences_ok;
    uint32_t sentences_bad_checksum;
    uint32_t sentences_overflow;
    uint32_t sentences_unsupported;

    bool fresh_fix_ready;
    GTU7_Fix latest;
} GTU7;

bool GTU7_Init(GTU7* dev, UART_HandleTypeDef* huart);
void GTU7_ResetParser(GTU7* dev);

bool GTU7_AttachDMARxBuffer(GTU7* dev, uint8_t* dma_rx_buffer, uint16_t dma_rx_buffer_len);
HAL_StatusTypeDef GTU7_StartRxDMA(GTU7* dev);
void GTU7_ProcessDMARing(GTU7* dev);

void GTU7_ProcessByte(GTU7* dev, uint8_t byte_in);
void GTU7_ProcessBuffer(GTU7* dev, const uint8_t* data, uint16_t len);

bool GTU7_ParseSentence(GTU7* dev, const char* sentence);
bool GTU7_GetLatest(const GTU7* dev, GTU7_Fix* out);
bool GTU7_HasFreshFix(const GTU7* dev);
void GTU7_ClearFreshFlag(GTU7* dev);

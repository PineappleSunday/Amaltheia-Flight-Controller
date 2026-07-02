#include "bme220.h"
#include <string.h>

static uint8_t bme220_checksum_u8(const uint8_t* data, uint8_t len_without_checksum) {
    uint16_t sum = 0u;
    uint8_t i = 0u;
    for (i = 0u; i < len_without_checksum; i++) {
        sum = (uint16_t)(sum + data[i]);
    }
    return (uint8_t)(sum & 0xFFu);
}

static uint16_t bme220_u16_le(const uint8_t* p) {
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static uint32_t bme220_u24_le(const uint8_t* p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16);
}

static uint32_t bme220_u32_le(const uint8_t* p) {
    return (uint32_t)p[0] |
           ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) |
           ((uint32_t)p[3] << 24);
}

bool BME220_ParseFrame0(const uint8_t frame[BME220_FRAME0_LEN_BYTES], BME220_Frame0* out) {
    if ((frame == NULL) || (out == NULL)) return false;
    if (frame[0] != BME220_FRAME_HEADER) return false;
    if (frame[1] != BME220_FUNC_OUTPUT_FRAME0) return false;
    if (bme220_checksum_u8(frame, (uint8_t)(BME220_FRAME0_LEN_BYTES - 1u)) != frame[BME220_FRAME0_LEN_BYTES - 1u]) {
        return false;
    }

    out->reserved0 = frame[2];
    out->node_id = frame[3];
    out->system_time_ms = bme220_u32_le(&frame[4]);
    out->distance_mm = bme220_u24_le(&frame[8]);
    out->distance_m = ((float)out->distance_mm) * 0.001f;
    out->distance_status = frame[11];
    out->signal_strength = bme220_u16_le(&frame[12]);
    out->reserved1 = frame[14];
    out->rx_tick_ms = HAL_GetTick();
    return true;
}

bool BME220_Init(BME220* dev, UART_HandleTypeDef* huart) {
    if ((dev == NULL) || (huart == NULL)) return false;

    memset(dev, 0, sizeof(*dev));
    dev->huart = huart;
    BME220_ResetParser(dev);
    return true;
}

void BME220_ResetParser(BME220* dev) {
    if (dev == NULL) return;

    dev->frame_index = 0u;
    dev->in_sync = false;
    dev->fresh_frame_ready = false;
}

bool BME220_AttachDMARxBuffer(BME220* dev, uint8_t* dma_rx_buffer, uint16_t dma_rx_buffer_len) {
    if ((dev == NULL) || (dma_rx_buffer == NULL) || (dma_rx_buffer_len < 2u)) return false;

    dev->dma_rx_buffer = dma_rx_buffer;
    dev->dma_rx_buffer_len = dma_rx_buffer_len;
    dev->dma_read_idx = 0u;
    return true;
}

HAL_StatusTypeDef BME220_StartRxDMA(BME220* dev) {
    if ((dev == NULL) || (dev->huart == NULL) || (dev->dma_rx_buffer == NULL) || (dev->dma_rx_buffer_len == 0u)) {
        return HAL_ERROR;
    }
    return HAL_UART_Receive_DMA(dev->huart, dev->dma_rx_buffer, dev->dma_rx_buffer_len);
}

void BME220_ProcessByte(BME220* dev, uint8_t byte_in) {
    if (dev == NULL) return;

    if (!dev->in_sync) {
        if (byte_in == BME220_FRAME_HEADER) {
            dev->in_sync = true;
            dev->frame_index = 0u;
            dev->frame_buf[dev->frame_index++] = byte_in;
        }
        return;
    }

    if (dev->frame_index >= BME220_FRAME0_LEN_BYTES) {
        dev->in_sync = false;
        dev->frame_index = 0u;
        dev->frames_bad_format++;
        return;
    }

    dev->frame_buf[dev->frame_index++] = byte_in;

    if ((dev->frame_index == 2u) && (dev->frame_buf[1] != BME220_FUNC_OUTPUT_FRAME0)) {
        dev->frames_bad_format++;
        if (byte_in == BME220_FRAME_HEADER) {
            dev->frame_buf[0] = BME220_FRAME_HEADER;
            dev->frame_index = 1u;
            dev->in_sync = true;
        } else {
            dev->frame_index = 0u;
            dev->in_sync = false;
        }
        return;
    }

    if (dev->frame_index < BME220_FRAME0_LEN_BYTES) return;

    if (bme220_checksum_u8(dev->frame_buf, (uint8_t)(BME220_FRAME0_LEN_BYTES - 1u)) != dev->frame_buf[BME220_FRAME0_LEN_BYTES - 1u]) {
        dev->frames_bad_checksum++;
    } else {
        BME220_Frame0 parsed = {0};
        if (BME220_ParseFrame0(dev->frame_buf, &parsed)) {
            dev->latest = parsed;
            dev->fresh_frame_ready = true;
            dev->frames_ok++;
        } else {
            dev->frames_bad_format++;
        }
    }

    dev->frame_index = 0u;
    dev->in_sync = false;
}

void BME220_ProcessBuffer(BME220* dev, const uint8_t* data, uint16_t len) {
    uint16_t i = 0u;
    if ((dev == NULL) || (data == NULL) || (len == 0u)) return;

    for (i = 0u; i < len; i++) {
        BME220_ProcessByte(dev, data[i]);
    }
}

void BME220_ProcessDMARing(BME220* dev) {
    uint16_t write_idx;
    if ((dev == NULL) || (dev->huart == NULL) || (dev->huart->hdmarx == NULL) ||
        (dev->dma_rx_buffer == NULL) || (dev->dma_rx_buffer_len == 0u)) {
        return;
    }

    write_idx = (uint16_t)((dev->dma_rx_buffer_len - __HAL_DMA_GET_COUNTER(dev->huart->hdmarx)) % dev->dma_rx_buffer_len);

    while (dev->dma_read_idx != write_idx) {
        BME220_ProcessByte(dev, dev->dma_rx_buffer[dev->dma_read_idx]);
        dev->dma_read_idx++;
        if (dev->dma_read_idx >= dev->dma_rx_buffer_len) dev->dma_read_idx = 0u;
    }
}

bool BME220_BuildQueryFrame0(uint8_t node_id, uint8_t out_frame[BME220_QUERY_FRAME0_LEN_BYTES]) {
    if (out_frame == NULL) return false;

    out_frame[0] = BME220_FRAME_HEADER;
    out_frame[1] = BME220_FUNC_QUERY_FRAME0;
    out_frame[2] = 0xFFu;
    out_frame[3] = 0xFFu;
    out_frame[4] = node_id;
    out_frame[5] = 0xFFu;
    out_frame[6] = 0xFFu;
    out_frame[7] = bme220_checksum_u8(out_frame, (uint8_t)(BME220_QUERY_FRAME0_LEN_BYTES - 1u));
    return true;
}

HAL_StatusTypeDef BME220_SendQueryFrame0(BME220* dev, uint8_t node_id, uint32_t timeout_ms) {
    uint8_t query[BME220_QUERY_FRAME0_LEN_BYTES];
    if ((dev == NULL) || (dev->huart == NULL)) return HAL_ERROR;
    if (!BME220_BuildQueryFrame0(node_id, query)) return HAL_ERROR;
    return HAL_UART_Transmit(dev->huart, query, BME220_QUERY_FRAME0_LEN_BYTES, timeout_ms);
}

bool BME220_GetLatest(const BME220* dev, BME220_Frame0* out) {
    if ((dev == NULL) || (out == NULL)) return false;
    if (dev->frames_ok == 0u) return false;
    *out = dev->latest;
    return true;
}

bool BME220_HasFreshFrame(const BME220* dev) {
    if (dev == NULL) return false;
    return dev->fresh_frame_ready;
}

void BME220_ClearFreshFlag(BME220* dev) {
    if (dev == NULL) return;
    dev->fresh_frame_ready = false;
}

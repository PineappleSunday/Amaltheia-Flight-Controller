#pragma once

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

#define DSHOT300_MIN_THROTTLE_RAW (0u)
#define DSHOT300_MAX_THROTTLE_RAW (2047u)

typedef struct {
    uint32_t channel;
    uint16_t raw_throttle;
    uint16_t packet;
    uint32_t updated_tick_ms;
    bool valid;
} ESC_D300_State;

// Encode a 16-bit DShot packet from a raw 11-bit throttle value.
// Telemetry request bit is fixed to 0 in this scaffold.
uint16_t ESC_D300_EncodePacket(uint16_t dshot_raw);

// Scaffold setter: builds/caches packet state only.
// This function intentionally does NOT drive timers or GPIO.
void ESC_D300_SetThrottle(uint32_t channel, uint16_t dshot_raw);

// Optional debug readback helpers.
bool ESC_D300_GetState(uint32_t channel, ESC_D300_State* out_state);
void ESC_D300_ResetState(void);

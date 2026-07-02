#include "dshot300.h"

typedef struct {
    uint32_t channel;
    ESC_D300_State state;
} ESC_D300_ChannelState;

static ESC_D300_ChannelState g_d300_state[4] = {
    { TIM_CHANNEL_1, { TIM_CHANNEL_1, 0u, 0u, 0u, false } },
    { TIM_CHANNEL_2, { TIM_CHANNEL_2, 0u, 0u, 0u, false } },
    { TIM_CHANNEL_3, { TIM_CHANNEL_3, 0u, 0u, 0u, false } },
    { TIM_CHANNEL_4, { TIM_CHANNEL_4, 0u, 0u, 0u, false } }
};

static int32_t d300_channel_to_index(uint32_t channel) {
    uint8_t i = 0u;
    for (i = 0u; i < 4u; i++) {
        if (g_d300_state[i].channel == channel) {
            return (int32_t)i;
        }
    }
    return -1;
}

uint16_t ESC_D300_EncodePacket(uint16_t dshot_raw) {
    uint16_t clamped = dshot_raw;
    uint16_t payload;
    uint16_t checksum;
    uint16_t packet;

    if (clamped > DSHOT300_MAX_THROTTLE_RAW) clamped = DSHOT300_MAX_THROTTLE_RAW;

    // 12-bit payload: [11:1]=throttle, [0]=telemetry request (fixed 0 here).
    payload = (uint16_t)(clamped << 1);
    checksum = (uint16_t)((payload ^ (payload >> 4) ^ (payload >> 8)) & 0x000Fu);
    packet = (uint16_t)((payload << 4) | checksum);
    return packet;
}

void ESC_D300_SetThrottle(uint32_t channel, uint16_t dshot_raw) {
    int32_t idx = d300_channel_to_index(channel);
    uint16_t clamped = dshot_raw;

    if (idx < 0) return;
    if (clamped > DSHOT300_MAX_THROTTLE_RAW) clamped = DSHOT300_MAX_THROTTLE_RAW;

    // Scaffold only: cache encoded packet/state. No hardware actuation here.
    g_d300_state[idx].state.channel = channel;
    g_d300_state[idx].state.raw_throttle = clamped;
    g_d300_state[idx].state.packet = ESC_D300_EncodePacket(clamped);
    g_d300_state[idx].state.updated_tick_ms = HAL_GetTick();
    g_d300_state[idx].state.valid = true;
}

bool ESC_D300_GetState(uint32_t channel, ESC_D300_State* out_state) {
    int32_t idx;
    if (out_state == NULL) return false;

    idx = d300_channel_to_index(channel);
    if (idx < 0) return false;
    if (!g_d300_state[idx].state.valid) return false;

    *out_state = g_d300_state[idx].state;
    return true;
}

void ESC_D300_ResetState(void) {
    uint8_t i = 0u;
    for (i = 0u; i < 4u; i++) {
        g_d300_state[i].state.channel = g_d300_state[i].channel;
        g_d300_state[i].state.raw_throttle = 0u;
        g_d300_state[i].state.packet = 0u;
        g_d300_state[i].state.updated_tick_ms = 0u;
        g_d300_state[i].state.valid = false;
    }
}

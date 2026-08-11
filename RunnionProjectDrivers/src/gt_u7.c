#include "gt_u7.h"
#include <stdlib.h>
#include <string.h>

#define GTU7_KNOTS_TO_MPS (0.514444444f)

static bool field_matches(const char* s, const char* a, const char* b)
{
    return ((strcmp(s, a) == 0) || (strcmp(s, b) == 0));
}

static uint8_t hex_nibble(char c)
{
    if ((c >= '0') && (c <= '9')) return (uint8_t)(c - '0');
    if ((c >= 'A') && (c <= 'F')) return (uint8_t)(c - 'A' + 10);
    if ((c >= 'a') && (c <= 'f')) return (uint8_t)(c - 'a' + 10);
    return 0xFFu;
}

static bool nmea_checksum_ok(const char* sentence)
{
    uint8_t calc = 0u;
    uint8_t got;
    const char* p;
    uint8_t hi;
    uint8_t lo;

    if ((sentence == NULL) || (sentence[0] != '$')) return false;

    p = &sentence[1];
    while ((*p != '\0') && (*p != '*')) {
        calc ^= (uint8_t)(*p);
        p++;
    }

    if ((p[0] != '*') || (p[1] == '\0') || (p[2] == '\0')) return false;

    hi = hex_nibble(p[1]);
    lo = hex_nibble(p[2]);
    if ((hi > 0x0Fu) || (lo > 0x0Fu)) return false;

    got = (uint8_t)((hi << 4) | lo);
    return calc == got;
}

static void strip_checksum(char* sentence)
{
    char* star = strchr(sentence, '*');
    if (star != NULL) {
        *star = '\0';
    }
}

static char* next_field(char** cursor)
{
    char* field;
    char* comma;

    if ((cursor == NULL) || (*cursor == NULL)) return NULL;

    field = *cursor;
    comma = strchr(field, ',');
    if (comma != NULL) {
        *comma = '\0';
        *cursor = comma + 1;
    } else {
        *cursor = NULL;
    }

    return field;
}

static double parse_latlon_deg(const char* value, const char* hemi)
{
    double raw;
    int deg;
    double minutes;
    double result;

    if ((value == NULL) || (value[0] == '\0') || (hemi == NULL) || (hemi[0] == '\0')) {
        return 0.0;
    }

    raw = strtod(value, NULL);
    deg = (int)(raw / 100.0);
    minutes = raw - ((double)deg * 100.0);
    result = (double)deg + (minutes / 60.0);

    if ((hemi[0] == 'S') || (hemi[0] == 'W')) {
        result = -result;
    }

    return result;
}

static void parse_utc_time(GTU7_Fix* fix, const char* hhmmss)
{
    uint32_t whole;
    const char* dot;
    uint16_t ms = 0u;

    if ((fix == NULL) || (hhmmss == NULL) || (strlen(hhmmss) < 6u)) return;

    whole = (uint32_t)strtoul(hhmmss, NULL, 10);
    fix->utc_hours = (uint8_t)(whole / 10000u);
    fix->utc_minutes = (uint8_t)((whole / 100u) % 100u);
    fix->utc_seconds = (uint8_t)(whole % 100u);

    dot = strchr(hhmmss, '.');
    if (dot != NULL) {
        uint16_t scale = 100u;
        dot++;
        while ((*dot >= '0') && (*dot <= '9') && (scale > 0u)) {
            ms = (uint16_t)(ms + (uint16_t)(*dot - '0') * scale);
            scale = (uint16_t)(scale / 10u);
            dot++;
        }
    }
    fix->utc_milliseconds = ms;
}

static void parse_date(GTU7_Fix* fix, const char* ddmmyy)
{
    uint32_t packed;
    uint8_t yy;

    if ((fix == NULL) || (ddmmyy == NULL) || (strlen(ddmmyy) < 6u)) return;

    packed = (uint32_t)strtoul(ddmmyy, NULL, 10);
    fix->date_day = (uint8_t)(packed / 10000u);
    fix->date_month = (uint8_t)((packed / 100u) % 100u);
    yy = (uint8_t)(packed % 100u);
    fix->date_year = (uint16_t)(2000u + yy);
}

static bool parse_gga(GTU7* dev, char* body)
{
    char* cursor = body;
    char* type = next_field(&cursor);
    char* time = next_field(&cursor);
    char* lat = next_field(&cursor);
    char* ns = next_field(&cursor);
    char* lon = next_field(&cursor);
    char* ew = next_field(&cursor);
    char* quality = next_field(&cursor);
    char* sats = next_field(&cursor);
    char* hdop = next_field(&cursor);
    char* altitude = next_field(&cursor);
    (void)next_field(&cursor); // altitude units
    char* geoid = next_field(&cursor);

    if ((dev == NULL) || !field_matches(type, "GPGGA", "GNGGA")) return false;

    parse_utc_time(&dev->latest, time);
    dev->latest.fix_quality = (uint8_t)strtoul((quality != NULL) ? quality : "0", NULL, 10);
    dev->latest.valid = (dev->latest.fix_quality > 0u);
    dev->latest.satellites = (uint8_t)strtoul((sats != NULL) ? sats : "0", NULL, 10);
    dev->latest.hdop = strtof((hdop != NULL) ? hdop : "0", NULL);
    dev->latest.latitude_deg = parse_latlon_deg(lat, ns);
    dev->latest.longitude_deg = parse_latlon_deg(lon, ew);
    dev->latest.altitude_m = strtof((altitude != NULL) ? altitude : "0", NULL);
    dev->latest.geoid_sep_m = strtof((geoid != NULL) ? geoid : "0", NULL);
    dev->latest.rx_tick_ms = HAL_GetTick();
    dev->fresh_fix_ready = true;
    return true;
}

static bool parse_rmc(GTU7* dev, char* body)
{
    char* cursor = body;
    char* type = next_field(&cursor);
    char* time = next_field(&cursor);
    char* status = next_field(&cursor);
    char* lat = next_field(&cursor);
    char* ns = next_field(&cursor);
    char* lon = next_field(&cursor);
    char* ew = next_field(&cursor);
    char* speed_knots = next_field(&cursor);
    char* course = next_field(&cursor);
    char* date = next_field(&cursor);

    if ((dev == NULL) || !field_matches(type, "GPRMC", "GNRMC")) return false;

    parse_utc_time(&dev->latest, time);
    parse_date(&dev->latest, date);
    dev->latest.valid = ((status != NULL) && (status[0] == 'A'));
    dev->latest.latitude_deg = parse_latlon_deg(lat, ns);
    dev->latest.longitude_deg = parse_latlon_deg(lon, ew);
    dev->latest.speed_mps = strtof((speed_knots != NULL) ? speed_knots : "0", NULL) * GTU7_KNOTS_TO_MPS;
    dev->latest.course_deg = strtof((course != NULL) ? course : "0", NULL);
    dev->latest.rx_tick_ms = HAL_GetTick();
    dev->fresh_fix_ready = true;
    return true;
}

bool GTU7_Init(GTU7* dev, UART_HandleTypeDef* huart)
{
    if ((dev == NULL) || (huart == NULL)) return false;

    memset(dev, 0, sizeof(*dev));
    dev->huart = huart;
    GTU7_ResetParser(dev);
    return true;
}

void GTU7_ResetParser(GTU7* dev)
{
    if (dev == NULL) return;

    dev->sentence_len = 0u;
    dev->collecting = false;
    dev->fresh_fix_ready = false;
}

bool GTU7_AttachDMARxBuffer(GTU7* dev, uint8_t* dma_rx_buffer, uint16_t dma_rx_buffer_len)
{
    if ((dev == NULL) || (dma_rx_buffer == NULL) || (dma_rx_buffer_len < 2u)) return false;

    dev->dma_rx_buffer = dma_rx_buffer;
    dev->dma_rx_buffer_len = dma_rx_buffer_len;
    dev->dma_read_idx = 0u;
    return true;
}

HAL_StatusTypeDef GTU7_StartRxDMA(GTU7* dev)
{
    if ((dev == NULL) || (dev->huart == NULL) || (dev->dma_rx_buffer == NULL) || (dev->dma_rx_buffer_len == 0u)) {
        return HAL_ERROR;
    }

    return HAL_UART_Receive_DMA(dev->huart, dev->dma_rx_buffer, dev->dma_rx_buffer_len);
}

void GTU7_ProcessByte(GTU7* dev, uint8_t byte_in)
{
    if (dev == NULL) return;

    if (byte_in == '$') {
        dev->collecting = true;
        dev->sentence_len = 0u;
        dev->sentence[dev->sentence_len++] = (char)byte_in;
        return;
    }

    if (!dev->collecting) return;

    if ((byte_in == '\r') || (byte_in == '\n')) {
        if (dev->sentence_len > 0u) {
            dev->sentence[dev->sentence_len] = '\0';
            (void)GTU7_ParseSentence(dev, dev->sentence);
        }
        dev->collecting = false;
        dev->sentence_len = 0u;
        return;
    }

    if (dev->sentence_len >= (GT_U7_MAX_SENTENCE_LEN - 1u)) {
        dev->sentences_overflow++;
        dev->collecting = false;
        dev->sentence_len = 0u;
        return;
    }

    dev->sentence[dev->sentence_len++] = (char)byte_in;
}

void GTU7_ProcessBuffer(GTU7* dev, const uint8_t* data, uint16_t len)
{
    uint16_t i;

    if ((dev == NULL) || (data == NULL) || (len == 0u)) return;

    for (i = 0u; i < len; i++) {
        GTU7_ProcessByte(dev, data[i]);
    }
}

void GTU7_ProcessDMARing(GTU7* dev)
{
    uint16_t write_idx;

    if ((dev == NULL) || (dev->huart == NULL) || (dev->huart->hdmarx == NULL) ||
        (dev->dma_rx_buffer == NULL) || (dev->dma_rx_buffer_len == 0u)) {
        return;
    }

    write_idx = (uint16_t)((dev->dma_rx_buffer_len - __HAL_DMA_GET_COUNTER(dev->huart->hdmarx)) % dev->dma_rx_buffer_len);

    while (dev->dma_read_idx != write_idx) {
        GTU7_ProcessByte(dev, dev->dma_rx_buffer[dev->dma_read_idx]);
        dev->dma_read_idx++;
        if (dev->dma_read_idx >= dev->dma_rx_buffer_len) dev->dma_read_idx = 0u;
    }
}

bool GTU7_ParseSentence(GTU7* dev, const char* sentence)
{
    char work[GT_U7_MAX_SENTENCE_LEN];
    char* body;
    bool parsed = false;

    if ((dev == NULL) || (sentence == NULL)) return false;

    if (!nmea_checksum_ok(sentence)) {
        dev->sentences_bad_checksum++;
        return false;
    }

    strncpy(work, sentence, sizeof(work) - 1u);
    work[sizeof(work) - 1u] = '\0';
    strip_checksum(work);
    body = (work[0] == '$') ? &work[1] : work;

    if ((strncmp(body, "GPGGA", 5u) == 0) || (strncmp(body, "GNGGA", 5u) == 0)) {
        parsed = parse_gga(dev, body);
    } else if ((strncmp(body, "GPRMC", 5u) == 0) || (strncmp(body, "GNRMC", 5u) == 0)) {
        parsed = parse_rmc(dev, body);
    }

    if (parsed) {
        dev->sentences_ok++;
    } else {
        dev->sentences_unsupported++;
    }

    return parsed;
}

bool GTU7_GetLatest(const GTU7* dev, GTU7_Fix* out)
{
    if ((dev == NULL) || (out == NULL)) return false;
    if (dev->sentences_ok == 0u) return false;
    *out = dev->latest;
    return true;
}

bool GTU7_HasFreshFix(const GTU7* dev)
{
    if (dev == NULL) return false;
    return dev->fresh_fix_ready;
}

void GTU7_ClearFreshFlag(GTU7* dev)
{
    if (dev == NULL) return;
    dev->fresh_fix_ready = false;
}

/*
 * crsf.h - TBS Crossfire / ExpressLRS serial protocol (CRSF).
 *
 * UART: 420000 baud 8N1, non-inverted, full duplex.
 * Frame:  [addr/sync][len][type][payload ...][crc8]
 *   len  = bytes after the len field = 1 (type) + payload + 1 (crc)
 *   crc8 = CRC-8/DVB-S2 (poly 0xD5, init 0) over type + payload
 * RC_CHANNELS_PACKED (0x16): 22 bytes = 16 channels x 11 bit, LSB first.
 *   172 -> 988 us, 992 -> 1500 us, 1811 -> 2012 us
 */
#ifndef CRSF_H
#define CRSF_H

#include <stdbool.h>
#include <stdint.h>

#define CRSF_BAUDRATE         420000u
#define CRSF_SYNC_FC          0xC8
#define CRSF_ADDR_RADIO       0xEA
#define CRSF_ADDR_TX_MODULE   0xEE
#define CRSF_ADDR_RECEIVER    0xEC
#define CRSF_MAX_FRAME        64
#define CRSF_MAX_PAYLOAD      (CRSF_MAX_FRAME - 4)
#define CRSF_NUM_CHANNELS     16
#define CRSF_FRAME_TIMEOUT_US 1500u /* partial-frame reset */

#define CRSF_CH_MIN    172
#define CRSF_CH_MID    992
#define CRSF_CH_MAX    1811

typedef enum {
    CRSF_TYPE_GPS = 0x02,
    CRSF_TYPE_BATTERY = 0x08,
    CRSF_TYPE_LINK_STATS = 0x14,
    CRSF_TYPE_RC_CHANNELS = 0x16,
    CRSF_TYPE_ATTITUDE = 0x1E,
    CRSF_TYPE_FLIGHT_MODE = 0x21,
} crsf_type_e;

typedef struct {
    uint8_t uplink_rssi1;      /* dBm * -1 */
    uint8_t uplink_rssi2;
    uint8_t uplink_lq;         /* % */
    int8_t  uplink_snr;        /* dB */
    uint8_t active_antenna;
    uint8_t rf_mode;
    uint8_t uplink_tx_power;
    uint8_t downlink_rssi;
    uint8_t downlink_lq;
    int8_t  downlink_snr;
} crsf_link_stats_t;

typedef struct {
    uint8_t buf[CRSF_MAX_FRAME];
    uint8_t pos;
    uint32_t last_byte_us;
    /* decoded data */
    uint16_t channels[CRSF_NUM_CHANNELS];
    crsf_link_stats_t link;
    uint32_t last_rc_us;
    bool rc_fresh;      /* set on each new RC frame, cleared by consumer */
    /* statistics */
    uint32_t frames_ok;
    uint32_t crc_errors;
    uint32_t framing_errors;
} crsf_parser_t;

uint8_t  crsf_crc8_update(uint8_t crc, uint8_t byte);
uint8_t  crsf_crc8(const uint8_t *data, int len);

void     crsf_parser_init(crsf_parser_t *p);
/* Feed one byte; returns the frame type of a completed valid frame, else 0. */
uint8_t  crsf_parse_byte(crsf_parser_t *p, uint8_t byte, uint32_t now_us);

void     crsf_unpack_channels(const uint8_t payload[22], uint16_t ch[CRSF_NUM_CHANNELS]);
void     crsf_pack_channels(const uint16_t ch[CRSF_NUM_CHANNELS], uint8_t payload[22]);
float    crsf_ch_to_us(uint16_t v);
float    crsf_ch_to_norm(uint16_t v); /* [-1, 1] */

/* Telemetry frame builders: return total frame length written to out. */
int crsf_build_frame(uint8_t *out, uint8_t type, const uint8_t *payload, int plen);
int crsf_build_rc_channels(uint8_t *out, const uint16_t ch[CRSF_NUM_CHANNELS]);
int crsf_build_attitude(uint8_t *out, float pitch_rad, float roll_rad, float yaw_rad);
int crsf_build_battery(uint8_t *out, float voltage, float current, uint32_t mah, uint8_t pct);
int crsf_build_flight_mode(uint8_t *out, const char *mode);

#endif

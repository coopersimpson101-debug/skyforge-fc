#include "crsf.h"

uint8_t crsf_crc8_update(uint8_t crc, uint8_t byte)
{
    crc ^= byte;
    for (int i = 0; i < 8; i++) {
        crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0xD5) : (uint8_t)(crc << 1);
    }
    return crc;
}

uint8_t crsf_crc8(const uint8_t *data, int len)
{
    uint8_t crc = 0;
    for (int i = 0; i < len; i++) {
        crc = crsf_crc8_update(crc, data[i]);
    }
    return crc;
}

void crsf_parser_init(crsf_parser_t *p)
{
    uint8_t *b = (uint8_t *)p;
    for (unsigned i = 0; i < sizeof(*p); i++) {
        b[i] = 0;
    }
    for (int i = 0; i < CRSF_NUM_CHANNELS; i++) {
        p->channels[i] = CRSF_CH_MID;
    }
}

void crsf_unpack_channels(const uint8_t payload[22], uint16_t ch[CRSF_NUM_CHANNELS])
{
    uint32_t acc = 0;
    int bits = 0, idx = 0;
    for (int i = 0; i < 22 && idx < CRSF_NUM_CHANNELS; i++) {
        acc |= (uint32_t)payload[i] << bits;
        bits += 8;
        while (bits >= 11 && idx < CRSF_NUM_CHANNELS) {
            ch[idx++] = (uint16_t)(acc & 0x7FF);
            acc >>= 11;
            bits -= 11;
        }
    }
}

void crsf_pack_channels(const uint16_t ch[CRSF_NUM_CHANNELS], uint8_t payload[22])
{
    uint32_t acc = 0;
    int bits = 0, out = 0;
    for (int i = 0; i < CRSF_NUM_CHANNELS; i++) {
        acc |= (uint32_t)(ch[i] & 0x7FF) << bits;
        bits += 11;
        while (bits >= 8) {
            payload[out++] = (uint8_t)(acc & 0xFF);
            acc >>= 8;
            bits -= 8;
        }
    }
}

float crsf_ch_to_us(uint16_t v)
{
    return ((float)v - (float)CRSF_CH_MID) * 0.625f + 1500.0f;
}

float crsf_ch_to_norm(uint16_t v)
{
    float n = ((float)v - (float)CRSF_CH_MID) / (float)(CRSF_CH_MAX - CRSF_CH_MID);
    return n < -1.0f ? -1.0f : (n > 1.0f ? 1.0f : n);
}

static void handle_frame(crsf_parser_t *p, uint32_t now_us)
{
    const uint8_t type = p->buf[2];
    const uint8_t *pl = &p->buf[3];
    const int plen = p->buf[1] - 2;
    if (type == CRSF_TYPE_RC_CHANNELS && plen == 22) {
        crsf_unpack_channels(pl, p->channels);
        p->last_rc_us = now_us;
        p->rc_fresh = true;
    } else if (type == CRSF_TYPE_LINK_STATS && plen >= 10) {
        p->link.uplink_rssi1 = pl[0];
        p->link.uplink_rssi2 = pl[1];
        p->link.uplink_lq = pl[2];
        p->link.uplink_snr = (int8_t)pl[3];
        p->link.active_antenna = pl[4];
        p->link.rf_mode = pl[5];
        p->link.uplink_tx_power = pl[6];
        p->link.downlink_rssi = pl[7];
        p->link.downlink_lq = pl[8];
        p->link.downlink_snr = (int8_t)pl[9];
    }
}

uint8_t crsf_parse_byte(crsf_parser_t *p, uint8_t byte, uint32_t now_us)
{
    if (p->pos > 0 && (uint32_t)(now_us - p->last_byte_us) > CRSF_FRAME_TIMEOUT_US) {
        p->pos = 0; /* stale partial frame */
        p->framing_errors++;
    }
    p->last_byte_us = now_us;

    if (p->pos == 0) {
        if (byte == CRSF_SYNC_FC || byte == CRSF_ADDR_TX_MODULE ||
            byte == CRSF_ADDR_RADIO || byte == CRSF_ADDR_RECEIVER) {
            p->buf[p->pos++] = byte;
        }
        return 0;
    }
    if (p->pos == 1) {
        if (byte < 2 || byte > CRSF_MAX_FRAME - 2) {
            p->pos = 0;
            p->framing_errors++;
            return 0;
        }
        p->buf[p->pos++] = byte;
        return 0;
    }
    p->buf[p->pos++] = byte;
    const int total = p->buf[1] + 2;
    if (p->pos < total) {
        return 0;
    }
    p->pos = 0;
    const uint8_t crc = crsf_crc8(&p->buf[2], p->buf[1] - 1);
    if (crc != p->buf[total - 1]) {
        p->crc_errors++;
        return 0;
    }
    p->frames_ok++;
    handle_frame(p, now_us);
    return p->buf[2];
}

int crsf_build_frame(uint8_t *out, uint8_t type, const uint8_t *payload, int plen)
{
    if (plen < 0 || plen > CRSF_MAX_PAYLOAD) {
        return 0;
    }
    out[0] = CRSF_SYNC_FC;
    out[1] = (uint8_t)(plen + 2);
    out[2] = type;
    for (int i = 0; i < plen; i++) {
        out[3 + i] = payload[i];
    }
    out[3 + plen] = crsf_crc8(&out[2], plen + 1);
    return plen + 4;
}

int crsf_build_rc_channels(uint8_t *out, const uint16_t ch[CRSF_NUM_CHANNELS])
{
    uint8_t pl[22];
    crsf_pack_channels(ch, pl);
    return crsf_build_frame(out, CRSF_TYPE_RC_CHANNELS, pl, 22);
}

static void put_be16(uint8_t *p, int16_t v)
{
    p[0] = (uint8_t)(((uint16_t)v) >> 8);
    p[1] = (uint8_t)(((uint16_t)v) & 0xFF);
}

static int16_t rad_to_crsf(float r)
{
    float v = r * 10000.0f;
    if (v > 32767.0f) v = 32767.0f;
    if (v < -32768.0f) v = -32768.0f;
    return (int16_t)(v >= 0.0f ? v + 0.5f : v - 0.5f);
}

int crsf_build_attitude(uint8_t *out, float pitch, float roll, float yaw)
{
    uint8_t pl[6];
    put_be16(&pl[0], rad_to_crsf(pitch));
    put_be16(&pl[2], rad_to_crsf(roll));
    put_be16(&pl[4], rad_to_crsf(yaw));
    return crsf_build_frame(out, CRSF_TYPE_ATTITUDE, pl, 6);
}

int crsf_build_battery(uint8_t *out, float voltage, float current, uint32_t mah, uint8_t pct)
{
    uint8_t pl[8];
    uint16_t dv = (uint16_t)(voltage * 10.0f + 0.5f);
    uint16_t da = (uint16_t)(current * 10.0f + 0.5f);
    pl[0] = (uint8_t)(dv >> 8); pl[1] = (uint8_t)dv;
    pl[2] = (uint8_t)(da >> 8); pl[3] = (uint8_t)da;
    pl[4] = (uint8_t)(mah >> 16); pl[5] = (uint8_t)(mah >> 8); pl[6] = (uint8_t)mah;
    pl[7] = pct;
    return crsf_build_frame(out, CRSF_TYPE_BATTERY, pl, 8);
}

int crsf_build_flight_mode(uint8_t *out, const char *mode)
{
    uint8_t pl[16];
    int n = 0;
    while (mode[n] && n < 14) {
        pl[n] = (uint8_t)mode[n];
        n++;
    }
    pl[n++] = 0; /* null terminated */
    return crsf_build_frame(out, CRSF_TYPE_FLIGHT_MODE, pl, n);
}

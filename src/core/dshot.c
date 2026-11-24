#include "dshot.h"

uint8_t dshot_crc(uint16_t v, bool bidir)
{
    uint16_t c = (uint16_t)(v ^ (v >> 4) ^ (v >> 8));
    if (bidir) {
        c = (uint16_t)~c;
    }
    return (uint8_t)(c & 0x0F);
}

uint16_t dshot_encode(uint16_t value, bool telemetry, bool bidir)
{
    uint16_t v = (uint16_t)(((value & 0x07FF) << 1) | (telemetry ? 1u : 0u));
    return (uint16_t)((v << 4) | dshot_crc(v, bidir));
}

bool dshot_decode(uint16_t packet, bool bidir, uint16_t *value, bool *telemetry)
{
    uint16_t v = (uint16_t)(packet >> 4);
    if ((packet & 0x0F) != dshot_crc(v, bidir)) {
        return false;
    }
    if (value) {
        *value = (uint16_t)(v >> 1);
    }
    if (telemetry) {
        *telemetry = (v & 1u) != 0;
    }
    return true;
}

uint16_t dshot_value_from_motor(float motor, float idle)
{
    if (motor < 0.0f) motor = 0.0f;
    if (motor > 1.0f) motor = 1.0f;
    if (idle < 0.0f) idle = 0.0f;
    if (idle > 0.5f) idle = 0.5f;
    float u = idle + (1.0f - idle) * motor;
    uint32_t v = DSHOT_MIN_THROTTLE + (uint32_t)(u * (float)DSHOT_RANGE + 0.5f);
    if (v > DSHOT_MAX_THROTTLE) v = DSHOT_MAX_THROTTLE;
    return (uint16_t)v;
}

int dshot_cmd_repeats(dshot_cmd_e cmd)
{
    switch (cmd) {
    case DSHOT_CMD_SPIN_DIRECTION_1:
    case DSHOT_CMD_SPIN_DIRECTION_2:
    case DSHOT_CMD_3D_MODE_OFF:
    case DSHOT_CMD_3D_MODE_ON:
    case DSHOT_CMD_SAVE_SETTINGS:
    case DSHOT_CMD_EXTENDED_TELEMETRY_ENABLE:
    case DSHOT_CMD_EXTENDED_TELEMETRY_DISABLE:
    case DSHOT_CMD_SPIN_DIRECTION_NORMAL:
    case DSHOT_CMD_SPIN_DIRECTION_REVERSED:
    case DSHOT_CMD_SIGNAL_LINE_TELEMETRY_DISABLE:
    case DSHOT_CMD_SIGNAL_LINE_TELEMETRY_ENABLE:
        return 10; /* spec: 6 minimum; 10 gives margin */
    default:
        return 1;
    }
}

dshot_timing_t dshot_timing(uint32_t timer_clk_hz, uint32_t bitrate)
{
    dshot_timing_t t;
    t.period = (timer_clk_hz + bitrate / 2) / bitrate;
    t.t0h = (t.period * 3 + 4) / 8;  /* 37.5 % */
    t.t1h = (t.period * 3 + 2) / 4;  /* 75 %   */
    return t;
}

void dshot_fill_ccr(uint16_t *buf, int stride, uint16_t packet, const dshot_timing_t *t)
{
    for (int i = 0; i < DSHOT_FRAME_BITS; i++) {
        const bool one = (packet & (0x8000u >> i)) != 0;
        buf[i * stride] = (uint16_t)(one ? t->t1h : t->t0h);
    }
    /* trailing low slots: line stays low -> inter-frame gap */
    buf[DSHOT_FRAME_BITS * stride] = 0;
    buf[(DSHOT_FRAME_BITS + 1) * stride] = 0;
}

void dshot_bitbang_fill(uint32_t *buf, const uint16_t *packets,
                        const uint16_t *pin_mask, int motors)
{
    for (int i = 0; i < DSHOT_BB_SLOTS; i++) {
        buf[i] = 0;
    }
    uint32_t all = 0;
    for (int m = 0; m < motors; m++) {
        all |= pin_mask[m];
    }
    for (int b = 0; b < DSHOT_FRAME_BITS; b++) {
        uint32_t zeros = 0, ones = 0;
        for (int m = 0; m < motors; m++) {
            if (packets[m] & (0x8000u >> b)) {
                ones |= pin_mask[m];
            } else {
                zeros |= pin_mask[m];
            }
        }
        uint32_t *slot = &buf[b * DSHOT_BB_SLOTS_PER_BIT];
        slot[0] = all;                    /* BSRR low half: set   */
        slot[3] = zeros << 16;            /* BSRR high half: reset */
        slot[6] = ones << 16;
    }
    /* final bit-time stays all zero (lines low) */
}

void dshot_cmdq_init(dshot_cmd_queue_t *cq)
{
    cq->head = 0;
    cq->count = 0;
}

bool dshot_cmdq_push(dshot_cmd_queue_t *cq, dshot_cmd_e cmd, uint8_t motor_mask)
{
    if (cq->count >= DSHOT_CMD_QUEUE_LEN || (int)cmd > DSHOT_CMD_MAX) {
        return false;
    }
    int idx = (cq->head + cq->count) % DSHOT_CMD_QUEUE_LEN;
    cq->q[idx].cmd = (uint8_t)cmd;
    cq->q[idx].motor_mask = motor_mask;
    cq->q[idx].repeats = (uint8_t)dshot_cmd_repeats(cmd);
    /* beeps need ~260 ms of silence before the next command; at a 1 kHz
     * command rate the gap is expressed in frames */
    cq->q[idx].gap_frames = (cmd >= DSHOT_CMD_BEEP1 && cmd <= DSHOT_CMD_BEEP5) ? 255 : 10;
    cq->count++;
    return true;
}

bool dshot_cmdq_busy(const dshot_cmd_queue_t *cq)
{
    return cq->count > 0;
}

bool dshot_cmdq_next(dshot_cmd_queue_t *cq, uint16_t packets[], int motors, bool bidir)
{
    if (cq->count == 0) {
        return false;
    }
    dshot_cmd_item_t *it = &cq->q[cq->head];
    const uint16_t stop = dshot_encode(DSHOT_CMD_MOTOR_STOP, false, bidir);
    if (it->repeats > 0) {
        const uint16_t pkt = dshot_encode(it->cmd, true, bidir);
        for (int m = 0; m < motors; m++) {
            packets[m] = (it->motor_mask & (1u << m)) ? pkt : stop;
        }
        it->repeats--;
    } else {
        for (int m = 0; m < motors; m++) {
            packets[m] = stop;
        }
        if (it->gap_frames > 0) {
            it->gap_frames--;
        }
        if (it->gap_frames == 0) {
            cq->head = (cq->head + 1) % DSHOT_CMD_QUEUE_LEN;
            cq->count--;
        }
    }
    return true;
}

/*
 * dshot.h - DShot digital ESC protocol: packet encoding, special
 * commands, timer/DMA bit-buffer and GPIO bit-bang buffer generation.
 *
 * Frame (16 bits, MSB first):
 *   [ 11-bit value | 1-bit telemetry request | 4-bit CRC ]
 *   value 0        = motor stop (disarmed)
 *   value 1..47    = special commands (send with telemetry bit = 1)
 *   value 48..2047 = throttle (2000 steps)
 *   CRC = (v ^ (v >> 4) ^ (v >> 8)) & 0xF, v = value<<1 | telem
 *   (bidirectional DShot inverts the CRC: ~(...) & 0xF)
 *
 * Bit timing (DShot600: 1.667 us per bit):
 *   '0' high for 37.5 % of the bit period, '1' high for 75 %.
 */
#ifndef DSHOT_H
#define DSHOT_H

#include <stdbool.h>
#include <stdint.h>

#define DSHOT_MIN_THROTTLE 48
#define DSHOT_MAX_THROTTLE 2047
#define DSHOT_RANGE        (DSHOT_MAX_THROTTLE - DSHOT_MIN_THROTTLE)
#define DSHOT_FRAME_BITS   16
#define DSHOT_DMA_SLOTS    (DSHOT_FRAME_BITS + 2) /* +2 low slots = frame gap */

typedef enum {
    DSHOT_CMD_MOTOR_STOP = 0,
    DSHOT_CMD_BEEP1 = 1,
    DSHOT_CMD_BEEP2 = 2,
    DSHOT_CMD_BEEP3 = 3,
    DSHOT_CMD_BEEP4 = 4,
    DSHOT_CMD_BEEP5 = 5,
    DSHOT_CMD_ESC_INFO = 6,
    DSHOT_CMD_SPIN_DIRECTION_1 = 7,
    DSHOT_CMD_SPIN_DIRECTION_2 = 8,
    DSHOT_CMD_3D_MODE_OFF = 9,
    DSHOT_CMD_3D_MODE_ON = 10,
    DSHOT_CMD_SETTINGS_REQUEST = 11,
    DSHOT_CMD_SAVE_SETTINGS = 12,
    DSHOT_CMD_EXTENDED_TELEMETRY_ENABLE = 13,
    DSHOT_CMD_EXTENDED_TELEMETRY_DISABLE = 14,
    DSHOT_CMD_SPIN_DIRECTION_NORMAL = 20,
    DSHOT_CMD_SPIN_DIRECTION_REVERSED = 21,
    DSHOT_CMD_LED0_ON = 22,
    DSHOT_CMD_LED1_ON = 23,
    DSHOT_CMD_LED2_ON = 24,
    DSHOT_CMD_LED3_ON = 25,
    DSHOT_CMD_LED0_OFF = 26,
    DSHOT_CMD_LED1_OFF = 27,
    DSHOT_CMD_LED2_OFF = 28,
    DSHOT_CMD_LED3_OFF = 29,
    DSHOT_CMD_AUDIO_STREAM_MODE_TOGGLE = 30,
    DSHOT_CMD_SILENT_MODE_TOGGLE = 31,
    DSHOT_CMD_SIGNAL_LINE_TELEMETRY_DISABLE = 32,
    DSHOT_CMD_SIGNAL_LINE_TELEMETRY_ENABLE = 33,
    DSHOT_CMD_MAX = 47
} dshot_cmd_e;

/* ---- packet ---------------------------------------------------------- */
uint8_t  dshot_crc(uint16_t value12, bool bidir);
uint16_t dshot_encode(uint16_t value, bool telemetry, bool bidir);
/* Returns true and fills value/telemetry if the CRC matches. */
bool     dshot_decode(uint16_t packet, bool bidir, uint16_t *value, bool *telemetry);
/* Motor command [0,1] -> DShot throttle value [48,2047] with idle offset. */
uint16_t dshot_value_from_motor(float motor, float idle);
/* Number of times a command must be repeated to be accepted by the ESC. */
int      dshot_cmd_repeats(dshot_cmd_e cmd);

/* ---- timer + DMA (one CCR value per bit) ------------------------------- */
typedef struct {
    uint32_t period;  /* timer ticks per bit (ARR + 1) */
    uint32_t t0h;     /* CCR value for a '0' bit */
    uint32_t t1h;     /* CCR value for a '1' bit */
} dshot_timing_t;

/* bitrate: 150000/300000/600000/1200000 */
dshot_timing_t dshot_timing(uint32_t timer_clk_hz, uint32_t bitrate);
/*
 * Writes DSHOT_DMA_SLOTS compare values for one motor into buf with the
 * given stride (stride = motor count for an interleaved DMA burst buffer).
 */
void dshot_fill_ccr(uint16_t *buf, int stride, uint16_t packet, const dshot_timing_t *t);

/* ---- GPIO bit-bang alternative ---------------------------------------- */
/*
 * Each bit is split into 8 slots driven by a timer-triggered DMA writing
 * to GPIOx->BSRR: slot 0 sets all motor pins, slot 3 resets pins sending
 * '0' (37.5 %), slot 6 resets pins sending '1' (75 %). Other slots are 0.
 * pin_mask[i] is the BSRR set mask of motor i (1 << pin).
 * buf must hold DSHOT_BB_SLOTS words.
 */
#define DSHOT_BB_SLOTS_PER_BIT 8
#define DSHOT_BB_SLOTS (DSHOT_FRAME_BITS * DSHOT_BB_SLOTS_PER_BIT + DSHOT_BB_SLOTS_PER_BIT)
void dshot_bitbang_fill(uint32_t *buf, const uint16_t *packets,
                        const uint16_t *pin_mask, int motors);

/* ---- command sequencer ------------------------------------------------ */
#define DSHOT_CMD_QUEUE_LEN 8
typedef struct {
    uint8_t cmd;
    uint8_t motor_mask;  /* bit i -> motor i, 0xFF = all */
    uint8_t repeats;
    uint8_t gap_frames;  /* zero frames sent after the command */
} dshot_cmd_item_t;

typedef struct {
    dshot_cmd_item_t q[DSHOT_CMD_QUEUE_LEN];
    int head, count;
} dshot_cmd_queue_t;

void dshot_cmdq_init(dshot_cmd_queue_t *cq);
bool dshot_cmdq_push(dshot_cmd_queue_t *cq, dshot_cmd_e cmd, uint8_t motor_mask);
bool dshot_cmdq_busy(const dshot_cmd_queue_t *cq);
/*
 * Produce the next frame's packet for every motor. While a command is
 * active its packet (telemetry bit set) goes to the selected motors and
 * MOTOR_STOP to the others. Returns false when the queue is idle (caller
 * then sends normal throttle values).
 */
bool dshot_cmdq_next(dshot_cmd_queue_t *cq, uint16_t packets[], int motors, bool bidir);

#endif

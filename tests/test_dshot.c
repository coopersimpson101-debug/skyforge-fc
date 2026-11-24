#include "../src/core/dshot.h"
#include "tinytest.h"

/* Reference vectors produced by an independent bit-string implementation
 * (see docs/dshot.md); 1046/0 -> 0x82C6 is the common textbook example. */
static const struct { uint16_t v; int t; uint16_t pkt, pkt_bidir; } vec[] = {
    {1046, 0, 0x82C6, 0x82C9},
    {48, 0, 0x0606, 0x0609},
    {0, 0, 0x0000, 0x000F},
    {2047, 0, 0xFFEE, 0xFFE1},
    {1000, 1, 0x7D1B, 0x7D14},
    {21, 1, 0x02B9, 0x02B6},
    {7, 1, 0x00FF, 0x00F0},
    {1500, 0, 0xBB88, 0xBB87},
};

static void test_reference_vectors(void)
{
    for (unsigned i = 0; i < sizeof(vec) / sizeof(vec[0]); i++) {
        CHECK_EQ_INT(dshot_encode(vec[i].v, vec[i].t, false), vec[i].pkt);
        CHECK_EQ_INT(dshot_encode(vec[i].v, vec[i].t, true), vec[i].pkt_bidir);
    }
}

static void test_roundtrip_and_crc(void)
{
    for (uint16_t v = 0; v < 2048; v++) {
        for (int t = 0; t < 2; t++) {
            uint16_t pkt = dshot_encode(v, t, false), out;
            bool tel;
            CHECK(dshot_decode(pkt, false, &out, &tel));
            if (out != v || tel != (bool)t) {
                CHECK(0);
            }
            /* any single bit flip in the payload must be detected */
            for (int b = 4; b < 16; b++) {
                if (dshot_decode((uint16_t)(pkt ^ (1u << b)), false, NULL, NULL)) {
                    CHECK(0);
                }
            }
        }
    }
}

static void test_throttle_mapping(void)
{
    CHECK_EQ_INT(dshot_value_from_motor(0.0f, 0.0f), 48);
    CHECK_EQ_INT(dshot_value_from_motor(1.0f, 0.0f), 2047);
    CHECK_EQ_INT(dshot_value_from_motor(2.0f, 0.0f), 2047);
    CHECK_EQ_INT(dshot_value_from_motor(-1.0f, 0.0f), 48);
    /* 5.5 % idle: 48 + 0.055*1999 = 157.9 -> 158 */
    CHECK_EQ_INT(dshot_value_from_motor(0.0f, 0.055f), 158);
    CHECK_EQ_INT(dshot_value_from_motor(0.5f, 0.0f), 1048);
}

static void test_timing_and_ccr_buffer(void)
{
    /* STM32F405 TIM8 on APB2: 168 MHz timer clock, DShot600 */
    dshot_timing_t t = dshot_timing(168000000u, 600000u);
    CHECK_EQ_INT(t.period, 280);
    CHECK_EQ_INT(t.t0h, 105);
    CHECK_EQ_INT(t.t1h, 210);
    dshot_timing_t t300 = dshot_timing(84000000u, 300000u);
    CHECK_EQ_INT(t300.period, 280);

    uint16_t buf[DSHOT_DMA_SLOTS * 4];
    for (int i = 0; i < DSHOT_DMA_SLOTS * 4; i++) buf[i] = 0xAAAA;
    uint16_t pkt = 0x82C6; /* 1000 0010 1100 0110 */
    dshot_fill_ccr(&buf[1], 4, pkt, &t); /* motor index 1 of 4, interleaved */
    const char *bits = "1000001011000110";
    for (int i = 0; i < 16; i++) {
        CHECK_EQ_INT(buf[1 + 4 * i], bits[i] == '1' ? 210 : 105);
        CHECK_EQ_INT(buf[0 + 4 * i], 0xAAAA); /* other motors untouched */
    }
    CHECK_EQ_INT(buf[1 + 4 * 16], 0);
    CHECK_EQ_INT(buf[1 + 4 * 17], 0);
}

static void test_bitbang(void)
{
    uint32_t bb[DSHOT_BB_SLOTS];
    uint16_t pk[2] = {0x8000, 0x0001}; /* motor0 first bit 1, motor1 last bit 1 */
    uint16_t pins[2] = {1u << 0, 1u << 5};
    dshot_bitbang_fill(bb, pk, pins, 2);
    /* bit 0: both set, motor1 ('0') resets at slot 3, motor0 ('1') at slot 6 */
    CHECK_EQ_INT(bb[0], 0x21);
    CHECK_EQ_INT(bb[3], 0x20u << 16);
    CHECK_EQ_INT(bb[6], 0x01u << 16);
    CHECK_EQ_INT(bb[1], 0);
    /* bit 15: motor0 '0', motor1 '1' */
    CHECK_EQ_INT(bb[15 * 8 + 3], 0x01u << 16);
    CHECK_EQ_INT(bb[15 * 8 + 6], 0x20u << 16);
    /* trailing slots idle */
    for (int i = 16 * 8; i < DSHOT_BB_SLOTS; i++) CHECK_EQ_INT(bb[i], 0);
}

static void test_command_queue(void)
{
    dshot_cmd_queue_t q;
    dshot_cmdq_init(&q);
    uint16_t pk[4];
    CHECK(!dshot_cmdq_next(&q, pk, 4, false));
    CHECK(dshot_cmdq_push(&q, DSHOT_CMD_SPIN_DIRECTION_REVERSED, 0x02));
    CHECK(!dshot_cmdq_push(&q, (dshot_cmd_e)48, 0x01)); /* not a command */
    int cmd_frames = 0, frames = 0;
    while (dshot_cmdq_next(&q, pk, 4, false)) {
        frames++;
        uint16_t v;
        bool tel;
        CHECK(dshot_decode(pk[1], false, &v, &tel));
        if (v == DSHOT_CMD_SPIN_DIRECTION_REVERSED) {
            cmd_frames++;
            CHECK(tel);                   /* commands need telemetry bit */
            CHECK_EQ_INT(pk[0], 0);       /* other motors get MOTOR_STOP */
        }
        if (frames > 1000) break;
    }
    CHECK_EQ_INT(cmd_frames, 10);
    CHECK(!dshot_cmdq_busy(&q));
}

int main(void)
{
    RUN(test_reference_vectors);
    RUN(test_roundtrip_and_crc);
    RUN(test_throttle_mapping);
    RUN(test_timing_and_ccr_buffer);
    RUN(test_bitbang);
    RUN(test_command_queue);
    return TT_SUMMARY();
}

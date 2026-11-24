/*
 * FC core state machine (arming, failsafe, bench test), configuration
 * storage, CLI, formatting and scheduler tests.
 */
#include <string.h>

#include "../src/core/cli.h"
#include "../src/core/crsf.h"
#include "../src/core/dshot.h"
#include "../src/core/fc.h"
#include "../src/core/fmt.h"
#include "../src/core/sched.h"
#include "tinytest.h"

/* ---- helpers ----------------------------------------------------------- */
static fc_config_t cfg;
static fc_t fc;
static uint32_t now;

static void setup(void)
{
    config_defaults(&cfg);
    cfg.gyro_rate_hz = 8000;
    cfg.pid_loop_hz = 4000;
    fc_init(&fc, &cfg);
    now = 1000;
}

/* run n gyro samples level & still, sending RC every 4 ms */
static void run(int n, const rc_command_t *rc, bool link)
{
    for (int i = 0; i < n; i++) {
        now += 125;
        if (link && rc && (now % 4000) < 125) fc_rc_update(&fc, rc, now);
        if (fc_gyro_sample(&fc, v3(0, 0, 0), v3(0, 0, -1))) fc_pid_step(&fc, now);
    }
}

static rc_command_t rc_make(float thr, bool arm)
{
    rc_command_t r = {0, 0, 0, thr, arm, false};
    return r;
}

/* ---- arming --------------------------------------------------------------- */
static void test_loop_rates(void)
{
    setup();
    CHECK_EQ_INT(fc.pid_denom, 2);
    CHECK_NEAR(fc.pid_hz, 4000, 1e-3);
    cfg.pid_loop_hz = 1000;
    fc_apply_config(&fc);
    CHECK_EQ_INT(fc.pid_denom, 8);
    int due = 0;
    for (int i = 0; i < 64; i++) due += fc_gyro_sample(&fc, v3(0, 0, 0), v3(0, 0, -1));
    CHECK_EQ_INT(due, 8);
}

static void test_arming_checks(void)
{
    setup();
    rc_command_t off = rc_make(0, false), on = rc_make(0, true);
    run(100, &off, true);
    CHECK(fc.arm_block & ARM_BLOCK_GYRO_CAL);   /* calibration takes 1 s */
    run(8000, &off, true);
    CHECK(fc.cal.done);
    CHECK_EQ_INT(fc.arm_block, 0);
    run(400, &on, true);
    CHECK(fc.armed);
    /* disarm by switch */
    run(400, &off, true);
    CHECK(!fc.armed);
    /* throttle high blocks arming */
    rc_command_t hi_on = rc_make(0.5f, true);
    run(400, &hi_on, true);
    CHECK(!fc.armed);
    CHECK(fc.arm_block & ARM_BLOCK_THROTTLE);
    /* throttle lowered while switch still on: needs switch cycle */
    run(400, &on, true);
    CHECK(!fc.armed);
    CHECK(fc.arm_block & ARM_BLOCK_ARM_SWITCH);
    run(400, &off, true);
    run(400, &on, true);
    CHECK(fc.armed);
}

static void test_arm_switch_on_at_boot(void)
{
    setup();
    rc_command_t on = rc_make(0, true), off = rc_make(0, false);
    run(9000, &on, true);   /* powered up with arm switch on */
    CHECK(!fc.armed);
    CHECK(fc.arm_block & ARM_BLOCK_ARM_SWITCH);
    run(400, &off, true);
    run(400, &on, true);
    CHECK(fc.armed);
}

static void test_not_level_blocks_arming(void)
{
    setup();
    rc_command_t off = rc_make(0, false), on = rc_make(0, true);
    /* tilted 40 deg: accel shows it */
    vec3_t acc = v3(0, -0.6427876f, -0.7660444f);
    for (int i = 0; i < 9000; i++) {
        now += 125;
        if ((now % 4000) < 125) fc_rc_update(&fc, (i < 8800) ? &off : &on, now);
        if (fc_gyro_sample(&fc, v3(0, 0, 0), acc)) fc_pid_step(&fc, now);
    }
    CHECK(!fc.armed);
    CHECK(fc.arm_block & ARM_BLOCK_ANGLE);
}

static void test_failsafe_drop_and_land(void)
{
    /* LAND: link loss -> levelled fs throttle -> disarm after land time */
    setup();
    cfg.fs_land_time_s = 1.0f;
    rc_command_t off = rc_make(0, false), on = rc_make(0, true), fly = rc_make(0.5f, true);
    run(9000, &off, true);
    run(400, &on, true);
    run(400, &fly, true);
    CHECK(fc.armed);
    run(8 * 200, &fly, false);           /* 200 ms without frames: still OK */
    CHECK_EQ_INT(fc.fs, FS_IDLE);
    run(8 * 200, &fly, false);           /* > 300 ms: failsafe */
    CHECK_EQ_INT(fc.fs, FS_LANDING);
    CHECK(fc.armed);
    CHECK_EQ_INT(fc.mode, MODE_ANGLE);
    CHECK_NEAR(fc.throttle, cfg.fs_throttle, 1e-6);
    run(8 * 1100, &fly, false);
    CHECK(!fc.armed);
    CHECK_EQ_INT(fc.fs, FS_DISARMED);
    /* link back with switch on -> stays disarmed and blocked */
    run(800, &on, true);
    CHECK(!fc.armed);
    CHECK(fc.arm_block & ARM_BLOCK_FAILSAFE);
    run(400, &off, true);                /* cycle switch -> recovered */
    CHECK_EQ_INT(fc.fs, FS_IDLE);
    run(400, &on, true);
    CHECK(fc.armed);

    /* DROP: immediate disarm */
    setup();
    cfg.fs_procedure = FS_PROC_DROP;
    run(9000, &off, true);
    run(400, &on, true);
    CHECK(fc.armed);
    run(8 * 400, &on, false);
    CHECK(!fc.armed);
    CHECK_EQ_INT(fc.fs, FS_DISARMED);
}

static void test_motor_outputs(void)
{
    setup();
    rc_command_t off = rc_make(0, false), on = rc_make(0, true);
    uint16_t d[4];
    run(9000, &off, true);
    fc_motor_dshot(&fc, d);
    for (int i = 0; i < 4; i++) CHECK_EQ_INT(d[i], DSHOT_CMD_MOTOR_STOP);
    run(400, &on, true);
    fc_motor_dshot(&fc, d);
    for (int i = 0; i < 4; i++) {
        CHECK(d[i] >= dshot_value_from_motor(0, cfg.motor_idle)); /* idle spin */
        CHECK(d[i] < 400);
    }
    /* bench test refused while armed, works disarmed and times out */
    CHECK(!fc_bench_motor(&fc, 0, 0.1f, now));
    run(400, &off, true);
    CHECK(fc_bench_motor(&fc, 2, 0.1f, now));
    run(8, &off, true);
    fc_motor_dshot(&fc, d);
    CHECK_EQ_INT(d[0], 0);
    CHECK(d[2] > 48);
    CHECK(fc.arm_block & ARM_BLOCK_BENCH);
    run(8 * 2100, &off, true);
    fc_motor_dshot(&fc, d);
    CHECK_EQ_INT(d[2], 0);
}

static void test_crsf_mapping(void)
{
    setup();
    uint16_t ch[16];
    for (int i = 0; i < 16; i++) ch[i] = CRSF_CH_MID;
    ch[0] = CRSF_CH_MAX;   /* full right roll */
    ch[2] = CRSF_CH_MIN;   /* throttle low */
    ch[4] = 1811;          /* AUX1 high = arm */
    ch[5] = 172;           /* AUX2 low = acro */
    rc_command_t rc;
    fc_rc_from_crsf(&cfg, ch, &rc);
    CHECK_NEAR(rc.roll, 1.0, 1e-6);
    CHECK_NEAR(rc.pitch, 0.0, 1e-6);
    CHECK(rc.throttle < 0.002f);
    CHECK(rc.arm_switch);
    CHECK(!rc.angle_mode);
    ch[5] = 1500;
    fc_rc_from_crsf(&cfg, ch, &rc);
    CHECK(rc.angle_mode);
}

/* ---- config storage ---------------------------------------------------- */
static uint8_t flash[4096];
static int fl_erase(void *c) { (void)c; memset(flash, 0xFF, sizeof(flash)); return 0; }
static int fl_write(void *c, uint32_t off, const void *d, uint32_t n)
{
    (void)c;
    const uint8_t *s = (const uint8_t *)d;
    for (uint32_t i = 0; i < n; i++) {
        flash[off + i] &= s[i]; /* NOR flash: bits can only be cleared */
    }
    return 0;
}
static int fl_read(void *c, uint32_t off, void *d, uint32_t n)
{
    (void)c;
    memcpy(d, flash + off, n);
    return 0;
}
static const config_storage_t storage = {fl_erase, fl_write, fl_read, NULL};

static void test_config_storage(void)
{
    fc_config_t a, b;
    config_defaults(&a);
    memset(flash, 0xFF, sizeof(flash));
    CHECK(!config_load(&b, &storage));          /* blank flash */
    a.pid_p[0] = 61.5f;
    a.ahrs_type = 1;
    CHECK(config_save(&a, &storage));
    memset(&b, 0, sizeof(b));
    CHECK(config_load(&b, &storage));
    CHECK_NEAR(b.pid_p[0], 61.5, 1e-6);
    CHECK_EQ_INT(b.ahrs_type, 1);
    flash[20] ^= 0x04;                           /* corrupt one bit */
    CHECK(!config_load(&b, &storage));
    /* CRC32 catalogue check value */
    CHECK_EQ_INT(crc32_update(0, "123456789", 9), 0xCBF43926u);
}

static void test_params(void)
{
    fc_config_t c;
    config_defaults(&c);
    const param_t *p = param_find("p_roll");
    CHECK(p != NULL);
    CHECK(param_set(&c, p, 70));
    CHECK_NEAR(c.pid_p[0], 70, 1e-6);
    CHECK(!param_set(&c, p, 999));               /* out of range */
    CHECK(param_find("nonexistent") == NULL);
    const param_t *u = param_find("pid_loop_hz");
    CHECK(param_set(&c, u, 8000));
    CHECK_EQ_INT(c.pid_loop_hz, 8000);
    for (int i = 0; i < param_count(); i++) {
        float v = param_get(&c, param_at(i));
        CHECK(v >= param_at(i)->min && v <= param_at(i)->max); /* defaults valid */
    }
}

/* ---- CLI ------------------------------------------------------------------ */
static char out[8192];
static void cli_out(void *ctx, const char *s)
{
    (void)ctx;
    size_t n = strlen(out);
    while (*s && n + 1 < sizeof(out)) out[n++] = *s++;
    out[n] = 0;
}
static int dshot_calls, dshot_last;
static bool fake_dshot(int cmd, uint8_t mask) { (void)mask; dshot_calls++; dshot_last = cmd; return true; }

static void type(cli_t *c, const char *s)
{
    while (*s) cli_input(c, *s++);
}

static void test_cli(void)
{
    setup();
    cli_t cli;
    cli_init(&cli, &fc, &cfg, &storage, cli_out, NULL);
    cli.dshot_cmd = fake_dshot;
    out[0] = 0;
    type(&cli, "help\r");                        /* ignored until '#' */
    CHECK(strlen(out) == 0);
    type(&cli, "#");
    CHECK(fc.cli_active);
    out[0] = 0;
    type(&cli, "set p_roll = 62\r");
    CHECK(strstr(out, "p_roll = 62.000") != NULL);
    CHECK_NEAR(cfg.pid_p[0], 62, 1e-6);
    CHECK_NEAR(fc.pid[0].g.kp, 62, 1e-6);        /* applied live */
    out[0] = 0;
    type(&cli, "set p_roll 9999\r");
    CHECK(strstr(out, "out of range") != NULL);
    out[0] = 0;
    type(&cli, "get rate_max\r");
    CHECK(strstr(out, "rate_max_roll = 670.000") != NULL);
    CHECK(strstr(out, "rate_max_yaw") != NULL);
    CHECK(strstr(out, "p_roll") == NULL);
    out[0] = 0;
    type(&cli, "dump\r");
    CHECK(strstr(out, "set ekf_gyro_noise = 0.002000") != NULL);
    out[0] = 0;
    memset(flash, 0xFF, sizeof(flash));
    type(&cli, "save\r");
    CHECK(strstr(out, "saved") != NULL);
    fc_config_t check;
    CHECK(config_load(&check, &storage));
    CHECK_NEAR(check.pid_p[0], 62, 1e-6);
    out[0] = 0;
    type(&cli, "dshot 21 2\r");
    CHECK_EQ_INT(dshot_calls, 1);
    CHECK_EQ_INT(dshot_last, 21);
    run(8, NULL, false); /* arming flags are refreshed by the PID step */
    out[0] = 0;
    type(&cli, "status\r");
    CHECK(strstr(out, "arming blocked by:") != NULL);
    CHECK(strstr(out, "CLI") != NULL);
    out[0] = 0;
    type(&cli, "# just a comment\r");
    CHECK(strstr(out, "error") == NULL);
    out[0] = 0;
    type(&cli, "bogus\r");
    CHECK(strstr(out, "unknown command") != NULL);
    type(&cli, "exit\r");
    CHECK(!fc.cli_active);
}

/* ---- fmt + sched ----------------------------------------------------- */
static void test_fmt(void)
{
    char b[32];
    fmt_float(b, sizeof(b), -12.3456f, 3);
    CHECK(strcmp(b, "-12.346") == 0);
    fmt_float(b, sizeof(b), 0.0005f, 6);
    CHECK(strcmp(b, "0.000500") == 0);
    fmt_int(b, sizeof(b), -2147483647 - 1);
    CHECK(strcmp(b, "-2147483648") == 0);
    float v;
    CHECK(fmt_parse_float("42", &v) && v == 42.0f);
    CHECK(fmt_parse_float("-0.25", &v) && v == -0.25f);
    CHECK(fmt_parse_float("1e-3", &v) && fabsf(v - 1e-3f) < 1e-9f);
    CHECK(!fmt_parse_float("abc", &v));
    CHECK(!fmt_parse_float("1.2.3", &v));
    CHECK(!fmt_parse_float("", &v));
}

static int task_runs;
static void task_fn(void *arg, uint32_t t) { (void)arg; (void)t; task_runs++; }

static void test_sched(void)
{
    loop_stats_t ls;
    loop_stats_init(&ls, 250);
    uint32_t t = 0xFFFFF000u; /* exercise 32-bit wrap */
    for (int i = 0; i < 100; i++) {
        t += (i == 50) ? 600 : 250;
        loop_stats_tick(&ls, t);
    }
    CHECK_EQ_INT(ls.overruns, 1);
    CHECK_EQ_INT(ls.max_us, 600);
    CHECK_EQ_INT(ls.min_us, 250);
    task_t tasks[1] = {{"t", task_fn, NULL, 10000, 0, 0}};
    task_runs = 0;
    for (uint32_t us = 0; us < 100000; us += 250) sched_run(tasks, 1, us, NULL);
    CHECK_EQ_INT(task_runs, 9); /* first run after one full period */
}

int main(void)
{
    RUN(test_loop_rates);
    RUN(test_arming_checks);
    RUN(test_arm_switch_on_at_boot);
    RUN(test_not_level_blocks_arming);
    RUN(test_failsafe_drop_and_land);
    RUN(test_motor_outputs);
    RUN(test_crsf_mapping);
    RUN(test_config_storage);
    RUN(test_params);
    RUN(test_cli);
    RUN(test_fmt);
    RUN(test_sched);
    return TT_SUMMARY();
}

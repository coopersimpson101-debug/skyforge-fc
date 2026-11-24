/*
 * main.c - skyforge-fc firmware entry point.
 *
 * Real-time path (driven by the gyro data-ready interrupt, 8 kHz):
 *     EXTI flag -> SPI read (14 B) -> fc_gyro_sample()  [notch + LPF]
 *     every pid_denom samples -> fc_pid_step()          [AHRS, angle, rate PID,
 *                                                        mixer, arming, failsafe]
 *                            -> DShot frame via TIM8 + DMA burst
 * Background (cooperative scheduler, between gyro samples):
 *     CRSF byte parser (every pass), CLI (1 kHz), CRSF telemetry (10 Hz),
 *     status LED (20 Hz).
 */
#include "../boards/board.h"
#include "../core/cli.h"
#include "../core/crsf.h"
#include "../core/dshot.h"
#include "../core/fc.h"
#include "../core/fmt.h"
#include "../core/sched.h"
#include "../drivers/imu.h"

static fc_config_t g_cfg;
static fc_t g_fc;
static imu_dev_t g_imu;
static crsf_parser_t g_crsf;
static cli_t g_cli;
static dshot_cmd_queue_t g_dshot_cmds;
static loop_stats_t g_loop;
static bool g_cfg_from_flash;

/* ---- hooks for the CLI -------------------------------------------------- */
static void cli_out(void *ctx, const char *s)
{
    (void)ctx;
    board_cli_write(s);
}

static bool cli_dshot(int cmd, uint8_t mask)
{
    if (g_fc.armed) {
        return false;
    }
    return dshot_cmdq_push(&g_dshot_cmds, (dshot_cmd_e)cmd, mask);
}

/* ---- background tasks ------------------------------------------------------ */
static void task_rx(void *arg, uint32_t now)
{
    (void)arg;
    uint8_t b;
    int budget = 64; /* bound the time spent here */
    while (budget-- > 0 && board_rx_read(&b)) {
        if (crsf_parse_byte(&g_crsf, b, now) == CRSF_TYPE_RC_CHANNELS) {
            rc_command_t rc;
            fc_rc_from_crsf(&g_cfg, g_crsf.channels, &rc);
            fc_rc_update(&g_fc, &rc, now);
            g_crsf.rc_fresh = false;
        }
    }
}

static void task_cli(void *arg, uint32_t now)
{
    (void)arg;
    (void)now;
    uint8_t c;
    int budget = 32;
    while (budget-- > 0 && board_cli_read(&c)) {
        cli_input(&g_cli, (char)c);
    }
}

static void task_telemetry(void *arg, uint32_t now)
{
    (void)arg;
    (void)now;
    uint8_t frame[CRSF_MAX_FRAME];
    float r, p, y;
    q_to_euler(g_fc.q, &r, &p, &y);
    int n = crsf_build_attitude(frame, p, r, y);
    board_rx_write(frame, n);
    char mode[16] = {0};
    if (g_fc.fs != FS_IDLE) {
        fmt_append(mode, sizeof(mode), "!FS!");
    } else {
        fmt_append(mode, sizeof(mode), g_fc.mode == MODE_ANGLE ? "ANGL" : "ACRO");
        if (!g_fc.armed) fmt_append(mode, sizeof(mode), "*");
    }
    n = crsf_build_flight_mode(frame, mode);
    board_rx_write(frame, n);
}

static void task_led(void *arg, uint32_t now)
{
    (void)arg;
    static uint32_t phase;
    phase++;
    bool on;
    if (g_fc.armed) {
        on = true;                                   /* solid: armed */
    } else if (!g_fc.imu_ok) {
        on = (phase % 2) == 0;                       /* fast blink: no IMU */
    } else if (g_fc.arm_block) {
        on = (phase % 10) < 2;                       /* short blip: blocked */
    } else {
        on = (phase % 10) < 5;                       /* slow blink: ready */
    }
    (void)now;
    board_led(on);
}

static task_t g_tasks[] = {
    {"rx", task_rx, 0, 0, 0, 0},
    {"cli", task_cli, 0, 1000, 0, 0},
    {"telemetry", task_telemetry, 0, 100000, 0, 0},
    {"led", task_led, 0, 50000, 0, 0},
};

/* ---- real-time step ---------------------------------------------------------- */
static void gyro_step(void)
{
    vec3_t gyro, accel;
    const uint32_t now = board_micros();
    loop_stats_tick(&g_loop, now);
    if (!imu_read(&g_imu, &gyro, &accel)) {
        return;
    }
    if (!fc_gyro_sample(&g_fc, gyro, accel)) {
        return;
    }
    fc_pid_step(&g_fc, now);

    uint16_t packets[BOARD_MOTORS];
    if (!g_fc.armed && dshot_cmdq_next(&g_dshot_cmds, packets, BOARD_MOTORS, false)) {
        /* special command in progress (beep, direction, save ...) */
    } else {
        uint16_t values[BOARD_MOTORS];
        fc_motor_dshot(&g_fc, values);
        for (int i = 0; i < BOARD_MOTORS; i++) {
            packets[i] = dshot_encode(values[i], false, false);
        }
    }
    board_dshot_write(packets);
}

int main(void)
{
    board_init();
    board_cli_write("\r\nskyforge-fc booting on ");
    board_cli_write(board_name());
    board_cli_write(board_clock_ok() ? " (HSE 8 MHz, 168 MHz)\r\n" : " (HSE FAILED - HSI)\r\n");

    config_defaults(&g_cfg);
    g_cfg_from_flash = config_load(&g_cfg, board_config_storage());
    board_cli_write(g_cfg_from_flash ? "config: loaded from flash\r\n" : "config: defaults\r\n");

    const bool imu_ok = imu_init(&g_imu, board_imu_bus(), (imu_align_e)g_cfg.imu_align);
    board_cli_write("imu: ");
    board_cli_write(imu_name(g_imu.type));
    board_cli_write(imu_ok ? "\r\n" : " (init FAILED)\r\n");

    g_cfg.gyro_rate_hz = (uint16_t)g_imu.odr_hz; /* loop is clocked by the IMU */
    fc_init(&g_fc, &g_cfg);
    g_fc.imu_ok = imu_ok;
    crsf_parser_init(&g_crsf);
    dshot_cmdq_init(&g_dshot_cmds);
    loop_stats_init(&g_loop, 1000000u / g_imu.odr_hz);

    cli_init(&g_cli, &g_fc, &g_cfg, board_config_storage(), cli_out, 0);
    g_cli.reboot = board_reboot;
    g_cli.dshot_cmd = cli_dshot;
    g_cli.micros = board_micros;
    g_cli.imu_name = imu_name(g_imu.type);
    g_cli.loop = &g_loop;

    board_dshot_init((uint32_t)g_cfg.dshot_rate * 1000u);
    if (imu_ok) {
        board_gyro_irq_enable();
    }
    board_cli_write("ready. send '#' for the CLI. PROPS OFF ON THE BENCH!\r\n");

    for (;;) {
        if (board_gyro_ready()) {
            gyro_step();
        }
        sched_run(g_tasks, (int)(sizeof(g_tasks) / sizeof(g_tasks[0])), board_micros(),
                  board_micros);
    }
}

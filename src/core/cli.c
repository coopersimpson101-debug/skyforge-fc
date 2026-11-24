#include "cli.h"
#include "fmt.h"

static void puts_(cli_t *c, const char *s)
{
    c->out(c->out_ctx, s);
}

static void print_param(cli_t *c, const param_t *p, bool as_set)
{
    char buf[80];
    buf[0] = 0;
    if (as_set) {
        fmt_append(buf, sizeof(buf), "set ");
    }
    fmt_append(buf, sizeof(buf), p->name);
    fmt_append(buf, sizeof(buf), " = ");
    char num[24];
    float v = param_get(c->cfg, p);
    if (p->type == PT_FLOAT) {
        fmt_float(num, sizeof(num), v, (v != 0.0f && fm_absf(v) < 0.01f) ? 6 : 3);
    } else {
        fmt_int(num, sizeof(num), (int32_t)v);
    }
    fmt_append(buf, sizeof(buf), num);
    fmt_append(buf, sizeof(buf), "\r\n");
    puts_(c, buf);
}

static bool starts_with(const char *s, const char *prefix)
{
    while (*prefix) {
        if (*s++ != *prefix++) {
            return false;
        }
    }
    return true;
}

/* split line into whitespace separated tokens (in place) */
static int tokenize(char *line, char *argv[], int max)
{
    int n = 0;
    char *p = line;
    while (*p && n < max) {
        while (*p == ' ' || *p == '\t' || *p == '=') {
            *p++ = 0;
        }
        if (!*p) break;
        argv[n++] = p;
        while (*p && *p != ' ' && *p != '\t' && *p != '=') {
            p++;
        }
    }
    return n;
}

static void cmd_status(cli_t *c)
{
    fc_t *fc = c->fc;
    char buf[160], num[24];
    float r, p, y;
    q_to_euler(fc->q, &r, &p, &y);

    buf[0] = 0;
    fmt_append(buf, sizeof(buf), "imu: ");
    fmt_append(buf, sizeof(buf), c->imu_name ? c->imu_name : "?");
    fmt_append(buf, sizeof(buf), fc->cal.done ? " (calibrated)" : " (calibrating)");
    fmt_append(buf, sizeof(buf), "  ahrs: ");
    fmt_append(buf, sizeof(buf), fc->ahrs.type == AHRS_EKF ? "EKF" : "Madgwick");
    fmt_append(buf, sizeof(buf), "\r\n");
    puts_(c, buf);

    buf[0] = 0;
    fmt_append(buf, sizeof(buf), "attitude deg: roll ");
    fmt_float(num, sizeof(num), r * FM_RAD2DEG, 1); fmt_append(buf, sizeof(buf), num);
    fmt_append(buf, sizeof(buf), " pitch ");
    fmt_float(num, sizeof(num), p * FM_RAD2DEG, 1); fmt_append(buf, sizeof(buf), num);
    fmt_append(buf, sizeof(buf), " yaw ");
    fmt_float(num, sizeof(num), y * FM_RAD2DEG, 1); fmt_append(buf, sizeof(buf), num);
    fmt_append(buf, sizeof(buf), "\r\n");
    puts_(c, buf);

    buf[0] = 0;
    fmt_append(buf, sizeof(buf), "loop: gyro ");
    fmt_int(num, sizeof(num), (int32_t)fc->gyro_hz); fmt_append(buf, sizeof(buf), num);
    fmt_append(buf, sizeof(buf), " Hz, pid ");
    fmt_int(num, sizeof(num), (int32_t)fc->pid_hz); fmt_append(buf, sizeof(buf), num);
    fmt_append(buf, sizeof(buf), " Hz");
    if (c->loop && c->loop->count) {
        fmt_append(buf, sizeof(buf), ", avg ");
        fmt_float(num, sizeof(num), c->loop->avg_us, 1); fmt_append(buf, sizeof(buf), num);
        fmt_append(buf, sizeof(buf), " us, max ");
        fmt_uint(num, sizeof(num), c->loop->max_us); fmt_append(buf, sizeof(buf), num);
        fmt_append(buf, sizeof(buf), " us, overruns ");
        fmt_uint(num, sizeof(num), c->loop->overruns); fmt_append(buf, sizeof(buf), num);
    }
    fmt_append(buf, sizeof(buf), "\r\n");
    puts_(c, buf);

    buf[0] = 0;
    fmt_append(buf, sizeof(buf), fc->armed ? "ARMED" : "disarmed");
    fmt_append(buf, sizeof(buf), "  mode: ");
    fmt_append(buf, sizeof(buf), fc->mode == MODE_ANGLE ? "ANGLE" : "ACRO");
    fmt_append(buf, sizeof(buf), "  arming blocked by: ");
    char flags[100];
    fc_arm_block_str(fc->arm_block, flags, sizeof(flags));
    fmt_append(buf, sizeof(buf), flags);
    fmt_append(buf, sizeof(buf), "\r\n");
    puts_(c, buf);
}

static bool parse_motor(const char *s, uint8_t *mask, int *idx)
{
    if (fmt_streq(s, "all")) {
        *mask = 0x0F;
        *idx = -1;
        return true;
    }
    float v;
    if (!fmt_parse_float(s, &v) || v < 1.0f || v > 4.0f) {
        return false;
    }
    *idx = (int)v - 1;
    *mask = (uint8_t)(1u << *idx);
    return true;
}

void cli_exec(cli_t *c, char *line)
{
    while (*line == ' ' || *line == '\t') {
        line++;
    }
    if (*line == '#') {
        return; /* comment line (lets CLI scripts be pasted verbatim) */
    }
    char *argv[6];
    const int argc = tokenize(line, argv, 6);
    if (argc == 0) {
        return;
    }
    const char *cmd = argv[0];

    if (fmt_streq(cmd, "help")) {
        puts_(c, "commands: help status get set dump save defaults motor dshot reboot bl exit\r\n");
    } else if (fmt_streq(cmd, "status")) {
        cmd_status(c);
    } else if (fmt_streq(cmd, "get") || fmt_streq(cmd, "dump")) {
        const bool as_set = fmt_streq(cmd, "dump");
        int shown = 0;
        for (int i = 0; i < param_count(); i++) {
            const param_t *p = param_at(i);
            if (argc < 2 || starts_with(p->name, argv[1])) {
                print_param(c, p, as_set);
                shown++;
            }
        }
        if (shown == 0) {
            puts_(c, "error: no such parameter\r\n");
        }
    } else if (fmt_streq(cmd, "set")) {
        if (c->fc->armed) {
            puts_(c, "error: disarm first\r\n");
            return;
        }
        if (argc < 3) {
            puts_(c, "usage: set <name> <value>\r\n");
            return;
        }
        const param_t *p = param_find(argv[1]);
        float v;
        if (!p) {
            puts_(c, "error: unknown parameter\r\n");
        } else if (!fmt_parse_float(argv[2], &v)) {
            puts_(c, "error: bad number\r\n");
        } else if (!param_set(c->cfg, p, v)) {
            puts_(c, "error: out of range\r\n");
        } else {
            fc_apply_config(c->fc);
            print_param(c, p, false);
        }
    } else if (fmt_streq(cmd, "save")) {
        if (c->fc->armed || !c->storage) {
            puts_(c, "error: cannot save now\r\n");
        } else if (config_save(c->cfg, c->storage)) {
            puts_(c, "saved\r\n");
        } else {
            puts_(c, "error: flash write failed\r\n");
        }
    } else if (fmt_streq(cmd, "defaults")) {
        config_defaults(c->cfg);
        fc_apply_config(c->fc);
        puts_(c, "defaults loaded (use 'save' to keep)\r\n");
    } else if (fmt_streq(cmd, "motor")) {
        uint8_t mask;
        int idx;
        float v;
        if (argc < 3 || !parse_motor(argv[1], &mask, &idx) || !fmt_parse_float(argv[2], &v)) {
            puts_(c, "usage: motor <1-4|all> <0-0.5>   (PROPS OFF!)\r\n");
        } else if (!fc_bench_motor(c->fc, idx, v, c->micros ? c->micros() : c->fc->now_us)) {
            puts_(c, "error: refused (armed?)\r\n");
        } else {
            puts_(c, "motor test running for 2 s\r\n");
        }
    } else if (fmt_streq(cmd, "dshot")) {
        uint8_t mask = 0x0F;
        int idx;
        float v;
        if (argc < 2 || !fmt_parse_float(argv[1], &v) || v < 1.0f || v > 47.0f ||
            (argc >= 3 && !parse_motor(argv[2], &mask, &idx))) {
            puts_(c, "usage: dshot <1-47> [1-4|all]\r\n");
        } else if (c->fc->armed || !c->dshot_cmd || !c->dshot_cmd((int)v, mask)) {
            puts_(c, "error: refused\r\n");
        } else {
            puts_(c, "dshot command queued\r\n");
        }
    } else if (fmt_streq(cmd, "reboot") || fmt_streq(cmd, "bl")) {
        if (c->fc->armed) {
            puts_(c, "error: disarm first\r\n");
        } else if (c->reboot) {
            puts_(c, "rebooting\r\n");
            c->reboot(fmt_streq(cmd, "bl"));
        }
    } else if (fmt_streq(cmd, "exit")) {
        c->active = false;
        c->fc->cli_active = false;
        puts_(c, "bye\r\n");
    } else {
        puts_(c, "error: unknown command (try 'help')\r\n");
    }
}

void cli_init(cli_t *cli, fc_t *fc, fc_config_t *cfg, const config_storage_t *st,
              void (*out)(void *ctx, const char *s), void *out_ctx)
{
    uint8_t *b = (uint8_t *)cli;
    for (unsigned i = 0; i < sizeof(*cli); i++) {
        b[i] = 0;
    }
    cli->fc = fc;
    cli->cfg = cfg;
    cli->storage = st;
    cli->out = out;
    cli->out_ctx = out_ctx;
}

void cli_input(cli_t *c, char ch)
{
    if (!c->active) {
        if (ch == '#' && !c->fc->armed) {
            c->active = true;
            c->fc->cli_active = true;
            c->len = 0;
            puts_(c, "\r\nskyforge-fc CLI - arming disabled. type 'help'\r\n# ");
        }
        return;
    }
    if (ch == '\r' || ch == '\n') {
        if (c->len > 0) {
            puts_(c, "\r\n");
            c->line[c->len] = 0;
            c->len = 0;
            cli_exec(c, c->line);
        }
        if (c->active) {
            puts_(c, "# ");
        }
        return;
    }
    if (ch == 0x08 || ch == 0x7F) { /* backspace */
        if (c->len > 0) {
            c->len--;
            puts_(c, "\b \b");
        }
        return;
    }
    if (c->len < CLI_LINE_MAX - 1 && ch >= 0x20 && ch < 0x7F) {
        c->line[c->len++] = ch;
        char echo[2] = {ch, 0};
        puts_(c, echo);
    }
}

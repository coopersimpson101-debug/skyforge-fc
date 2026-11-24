/*
 * cli.h - line based configuration console over UART.
 *
 * Enter the CLI by sending '#'. While the CLI is active arming is blocked.
 *   help                      list commands
 *   status                    sensors, attitude, arming flags, loop timing
 *   get [prefix]              print parameters (optionally filtered)
 *   set <name> [=] <value>    change a parameter (applied immediately)
 *   dump                      print all parameters as 'set' commands
 *   save                      write config to flash (CRC protected)
 *   defaults                  restore defaults (not saved until 'save')
 *   motor <1-4|all> <0-0.5>   bench spin for 2 s  -- PROPS OFF!
 *   dshot <cmd> [1-4|all]     send DShot special command (e.g. 21 reverse)
 *   reboot | bl               reboot (bl = into ROM DFU bootloader)
 *   exit                      leave CLI (re-enables arming)
 */
#ifndef CLI_H
#define CLI_H

#include "config.h"
#include "fc.h"
#include "sched.h"

#define CLI_LINE_MAX 96

typedef struct cli_s {
    char line[CLI_LINE_MAX];
    int len;
    bool active;
    void (*out)(void *ctx, const char *s);
    void *out_ctx;
    fc_t *fc;
    fc_config_t *cfg;
    const config_storage_t *storage;
    /* optional platform hooks */
    void (*reboot)(bool bootloader);
    bool (*dshot_cmd)(int cmd, uint8_t motor_mask);
    uint32_t (*micros)(void);
    const char *imu_name;
    const loop_stats_t *loop;
} cli_t;

void cli_init(cli_t *cli, fc_t *fc, fc_config_t *cfg, const config_storage_t *st,
              void (*out)(void *ctx, const char *s), void *out_ctx);
/* Feed one received character. */
void cli_input(cli_t *cli, char c);
/* Execute one full command line (used by tests and by cli_input). */
void cli_exec(cli_t *cli, char *line);

#endif

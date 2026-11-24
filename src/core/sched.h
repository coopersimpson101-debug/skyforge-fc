/*
 * sched.h - loop timing statistics and a tiny cooperative scheduler for
 * the low-rate tasks (RX parsing, telemetry, CLI, LEDs).
 *
 * The hard real-time path (gyro -> PID -> DShot) is driven by the gyro
 * data-ready interrupt; everything else runs from sched_run() in the
 * main loop between gyro samples.
 */
#ifndef SCHED_H
#define SCHED_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint32_t expected_us;
    uint32_t last_us;
    uint32_t min_us, max_us;
    uint32_t overruns;   /* period > 1.5 x expected */
    uint32_t count;
    float avg_us;        /* exponential average */
    bool started;
} loop_stats_t;

void loop_stats_init(loop_stats_t *s, uint32_t expected_us);
void loop_stats_tick(loop_stats_t *s, uint32_t now_us);

typedef struct {
    const char *name;
    void (*fn)(void *arg, uint32_t now_us);
    void *arg;
    uint32_t period_us;
    uint32_t last_us;
    uint32_t max_exec_us;
} task_t;

/* Runs every task whose period elapsed. micros() is used to time tasks.
 * Returns the number of tasks executed. */
int sched_run(task_t *tasks, int n, uint32_t now_us, uint32_t (*micros)(void));

#endif

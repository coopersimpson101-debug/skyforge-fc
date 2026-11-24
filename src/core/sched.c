#include "sched.h"

void loop_stats_init(loop_stats_t *s, uint32_t expected_us)
{
    s->expected_us = expected_us;
    s->last_us = 0;
    s->min_us = 0xFFFFFFFFu;
    s->max_us = 0;
    s->overruns = 0;
    s->count = 0;
    s->avg_us = (float)expected_us;
    s->started = false;
}

void loop_stats_tick(loop_stats_t *s, uint32_t now_us)
{
    if (!s->started) {
        s->started = true;
        s->last_us = now_us;
        return;
    }
    const uint32_t dt = now_us - s->last_us; /* wrap-safe */
    s->last_us = now_us;
    if (dt < s->min_us) s->min_us = dt;
    if (dt > s->max_us) s->max_us = dt;
    if (2u * dt > 3u * s->expected_us) s->overruns++;
    s->avg_us += 0.01f * ((float)dt - s->avg_us);
    s->count++;
}

int sched_run(task_t *tasks, int n, uint32_t now_us, uint32_t (*micros)(void))
{
    int ran = 0;
    for (int i = 0; i < n; i++) {
        task_t *t = &tasks[i];
        if ((uint32_t)(now_us - t->last_us) >= t->period_us) {
            t->last_us = now_us;
            const uint32_t t0 = micros ? micros() : now_us;
            t->fn(t->arg, now_us);
            if (micros) {
                const uint32_t ex = micros() - t0;
                if (ex > t->max_exec_us) t->max_exec_us = ex;
            }
            ran++;
        }
    }
    return ran;
}

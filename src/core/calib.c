#include "calib.h"

static void reset_window(gyro_cal_t *c)
{
    for (int i = 0; i < 3; i++) {
        c->mean[i] = 0.0f;
        c->m2[i] = 0.0f;
    }
    c->n = 0;
}

void gyro_cal_start(gyro_cal_t *c, uint32_t samples, float max_std_rads)
{
    reset_window(c);
    c->target = samples > 10 ? samples : 10;
    c->max_std = max_std_rads;
    c->bias = v3(0.0f, 0.0f, 0.0f);
    c->done = false;
    c->restarts = 0;
}

bool gyro_cal_add(gyro_cal_t *c, vec3_t g)
{
    if (c->done) {
        return true;
    }
    /* Welford's online algorithm: numerically stable in single precision
     * (no double-precision soft-float on the Cortex-M4) */
    const float v[3] = {g.x, g.y, g.z};
    c->n++;
    for (int i = 0; i < 3; i++) {
        const float d = v[i] - c->mean[i];
        c->mean[i] += d / (float)c->n;
        c->m2[i] += d * (v[i] - c->mean[i]);
    }
    if (c->n < c->target) {
        return false;
    }
    float mean[3];
    bool still = true;
    for (int i = 0; i < 3; i++) {
        const float var = c->m2[i] / (float)c->n;
        mean[i] = c->mean[i];
        if (var > c->max_std * c->max_std) {
            still = false;
        }
    }
    if (!still) {
        reset_window(c);
        c->restarts++;
        return false;
    }
    c->bias = v3(mean[0], mean[1], mean[2]);
    c->done = true;
    return true;
}

/*
 * mixer.c - quad-X mixer with airmode.
 *
 * Desaturation: if the spread (max - min) of the roll/pitch/yaw part
 * exceeds the full motor range, the rpy vector is scaled down uniformly
 * (preserving its direction = attitude authority). With airmode the
 * throttle is then shifted so that every motor lies in [0,1], i.e. the
 * controller keeps authority at zero throttle and at full throttle.
 * Without airmode outputs are simply clipped (classic behaviour).
 */
#include "mixer.h"
#include "fc_math.h"

void mixer_init_quadx(mixer_t *m, bool airmode, bool yaw_reversed)
{
    static const mixer_rule_t quadx[4] = {
        /* roll   pitch   yaw */
        {-1.0f, -1.0f, -1.0f}, /* 1 rear-right,  CW  */
        {-1.0f, +1.0f, +1.0f}, /* 2 front-right, CCW */
        {+1.0f, -1.0f, +1.0f}, /* 3 rear-left,   CCW */
        {+1.0f, +1.0f, -1.0f}, /* 4 front-left,  CW  */
    };
    m->motor_count = 4;
    for (int i = 0; i < 4; i++) {
        m->rules[i] = quadx[i];
        /* +-0.5 per motor: a PID demand of +-1 spans the full motor range */
        m->rules[i].roll *= 0.5f;
        m->rules[i].pitch *= 0.5f;
        m->rules[i].yaw *= 0.5f;
    }
    m->airmode = airmode;
    m->yaw_reversed = yaw_reversed;
    m->saturated = false;
    m->rpy_scale = 1.0f;
}

bool mixer_mix(mixer_t *m, float throttle, float roll, float pitch, float yaw,
               float out[])
{
    float rpy[MIXER_MAX_MOTORS];
    float mn = 1e9f, mx = -1e9f;
    const float ysign = m->yaw_reversed ? -1.0f : 1.0f;
    throttle = fm_clampf(throttle, 0.0f, 1.0f);

    for (int i = 0; i < m->motor_count; i++) {
        rpy[i] = roll * m->rules[i].roll + pitch * m->rules[i].pitch +
                 ysign * yaw * m->rules[i].yaw;
        mn = fm_minf(mn, rpy[i]);
        mx = fm_maxf(mx, rpy[i]);
    }
    const float range = mx - mn;
    bool sat = false;
    m->rpy_scale = 1.0f;
    if (range > 1.0f) {
        m->rpy_scale = 1.0f / range;
        for (int i = 0; i < m->motor_count; i++) {
            rpy[i] *= m->rpy_scale;
        }
        mn *= m->rpy_scale;
        mx *= m->rpy_scale;
        sat = true;
    }

    if (m->airmode) {
        /* shift throttle so that throttle+min >= 0 and throttle+max <= 1 */
        float lo = -mn, hi = 1.0f - mx;
        float t = fm_clampf(throttle, lo, hi);
        if (t != throttle) {
            sat = true;
        }
        for (int i = 0; i < m->motor_count; i++) {
            out[i] = fm_clampf(t + rpy[i], 0.0f, 1.0f);
        }
    } else {
        for (int i = 0; i < m->motor_count; i++) {
            float v = throttle + rpy[i];
            if (v < 0.0f || v > 1.0f) {
                sat = true;
            }
            out[i] = fm_clampf(v, 0.0f, 1.0f);
        }
    }
    m->saturated = sat;
    return sat;
}

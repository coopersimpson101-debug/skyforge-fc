/*
 * quad_sim.c - host-side rigid-body simulator around the real FC core.
 * See quad_sim.h for the model. Host only (uses libc/libm).
 */
#include "quad_sim.h"

#include <math.h>
#include <stdlib.h>

#include "../src/core/dshot.h"

static float urand(quad_sim_t *s)
{
    /* xorshift32 */
    uint32_t x = s->rng;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    s->rng = x;
    return ((float)(x >> 8) + 0.5f) / 16777216.0f;
}

static float nrand(quad_sim_t *s)
{
    float u1 = urand(s), u2 = urand(s);
    return sqrtf(-2.0f * logf(u1)) * cosf(6.2831853f * u2);
}

SIM_API quad_sim_t *sim_create(void)
{
    quad_sim_t *s = (quad_sim_t *)calloc(1, sizeof(quad_sim_t));
    if (s) {
        sim_reset(s, 1);
    }
    return s;
}

SIM_API void sim_destroy(quad_sim_t *s)
{
    free(s);
}

SIM_API void sim_reset(quad_sim_t *s, uint32_t seed)
{
    s->mass = 0.65f;
    s->ixx = 0.0022f;
    s->iyy = 0.0024f;
    s->izz = 0.0040f;
    s->arm_xy = 0.0795f;      /* 225 mm motor-to-motor diagonal */
    s->thrust_max = 10.0f;    /* N per motor (~6:1 thrust/weight) */
    s->kq = 0.015f;           /* m: yaw torque per newton of thrust */
    s->motor_tau = 0.025f;    /* s */
    s->drag_lin = 0.15f;
    s->drag_rot = 0.0004f;
    s->gyro_noise = 0.01f;
    s->accel_noise = 0.03f;
    s->gyro_bias = v3(0.010f, -0.008f, 0.005f);
    s->vib_amp = 0.0f;
    s->vib_hz = 260.0f;
    s->rc_period_us = 4000;   /* 250 Hz ELRS-like */

    s->pos = v3(0, 0, 0);
    s->vel = v3(0, 0, 0);
    s->w = v3(0, 0, 0);
    s->q = q_identity();
    for (int i = 0; i < 4; i++) s->s[i] = 0.0f;
    s->on_ground = true;
    s->dist_torque = v3(0, 0, 0);
    s->t = 0.0f;
    s->now_us = 1000;
    s->last_rc_us = 0;
    s->rng = seed ? seed : 1;
    s->link_up = true;

    config_defaults(&s->cfg);
    fc_init(&s->fc, &s->cfg);
    s->rc.roll = s->rc.pitch = s->rc.yaw = 0.0f;
    s->rc.throttle = 0.0f;
    s->rc.arm_switch = false;
    s->rc.angle_mode = false;
}

SIM_API int sim_set_param(quad_sim_t *s, const char *name, float value)
{
    const param_t *p = param_find(name);
    if (!p || !param_set(&s->cfg, p, value)) {
        return 0;
    }
    /* keep calibration across re-configuration */
    gyro_cal_t cal = s->fc.cal;
    fc_apply_config(&s->fc);
    s->fc.cal = cal;
    return 1;
}

SIM_API void sim_set_rc(quad_sim_t *s, float roll, float pitch, float yaw, float throttle,
                        int arm, int angle_mode)
{
    s->rc.roll = roll;
    s->rc.pitch = pitch;
    s->rc.yaw = yaw;
    s->rc.throttle = throttle;
    s->rc.arm_switch = arm != 0;
    s->rc.angle_mode = angle_mode != 0;
}

SIM_API void sim_set_link(quad_sim_t *s, int up)
{
    s->link_up = up != 0;
}

SIM_API void sim_set_disturbance(quad_sim_t *s, float tx, float ty, float tz)
{
    s->dist_torque = v3(tx, ty, tz);
}

SIM_API void sim_kick_rates(quad_sim_t *s, float p, float q, float r)
{
    s->w = v3_add(s->w, v3(p * FM_DEG2RAD, q * FM_DEG2RAD, r * FM_DEG2RAD));
}

SIM_API float sim_hover_throttle(const quad_sim_t *s)
{
    float sh = sqrtf(s->mass * FM_GRAVITY / (4.0f * s->thrust_max));
    float idle = s->cfg.motor_idle;
    return (sh - idle) / (1.0f - idle);
}

static void physics(quad_sim_t *s, const float u[4], float dt)
{
    /* motor positions (FRD): 1 RR, 2 FR, 3 RL, 4 FL; spin: CW, CCW, CCW, CW */
    const float a = s->arm_xy;
    const float rx[4] = {-a, a, -a, a};
    const float ry[4] = {a, a, -a, -a};
    const float spin[4] = {-1.0f, 1.0f, 1.0f, -1.0f}; /* reaction yaw sign */

    float thrust = 0.0f;
    vec3_t tau = s->dist_torque;
    for (int i = 0; i < 4; i++) {
        s->s[i] += (u[i] - s->s[i]) * (dt / s->motor_tau);
        const float T = s->thrust_max * s->s[i] * s->s[i];
        thrust += T;
        tau.x += -ry[i] * T;
        tau.y += rx[i] * T;
        tau.z += spin[i] * s->kq * T;
    }
    tau = v3_sub(tau, v3_scale(s->w, s->drag_rot));

    /* rotational dynamics: I w' = tau - w x I w */
    const vec3_t Iw = v3(s->ixx * s->w.x, s->iyy * s->w.y, s->izz * s->w.z);
    const vec3_t rhs = v3_sub(tau, v3_cross(s->w, Iw));
    const vec3_t wdot = v3(rhs.x / s->ixx, rhs.y / s->iyy, rhs.z / s->izz);

    /* translational dynamics */
    const vec3_t f_world = q_rotate(s->q, v3(0.0f, 0.0f, -thrust));
    vec3_t acc = v3_add(v3_scale(f_world, 1.0f / s->mass), v3(0.0f, 0.0f, FM_GRAVITY));
    acc = v3_sub(acc, v3_scale(s->vel, s->drag_lin / s->mass));

    if (s->on_ground) {
        if (acc.z < 0.0f) {
            s->on_ground = false; /* lift-off */
        } else {
            /* resting on the ground: no motion */
            s->vel = v3(0, 0, 0);
            s->w = v3(0, 0, 0);
            return;
        }
    }
    s->w = v3_add(s->w, v3_scale(wdot, dt));
    s->q = q_normalize(q_mul(s->q, q_from_rotvec(v3_scale(s->w, dt))));
    s->vel = v3_add(s->vel, v3_scale(acc, dt));
    s->pos = v3_add(s->pos, v3_scale(s->vel, dt));
    if (s->pos.z > 0.0f) {
        /* touch-down: level the craft and stop (crude but sufficient) */
        s->pos.z = 0.0f;
        s->vel = v3(0, 0, 0);
        s->w = v3(0, 0, 0);
        s->on_ground = true;
    }
}

static void sense(quad_sim_t *s, vec3_t *gyro, vec3_t *accel)
{
    float thr = 0.25f * (s->s[0] + s->s[1] + s->s[2] + s->s[3]);
    float vib = s->vib_amp * thr * sinf(6.2831853f * s->vib_hz * s->t);
    *gyro = v3(s->w.x + s->gyro_bias.x + s->gyro_noise * nrand(s) + vib,
               s->w.y + s->gyro_bias.y + s->gyro_noise * nrand(s) + 0.7f * vib,
               s->w.z + s->gyro_bias.z + s->gyro_noise * nrand(s));
    /* specific force: R^T (a - g e3) / g. Use the last kinematic accel
     * implied by thrust (on the ground: reaction cancels gravity). */
    float thrust = 0.0f;
    for (int i = 0; i < 4; i++) thrust += s->thrust_max * s->s[i] * s->s[i];
    vec3_t f;
    if (s->on_ground && thrust < s->mass * FM_GRAVITY) {
        f = v3_scale(q_body_down(s->q), -1.0f);
    } else {
        /* in flight: specific force = (thrust + drag) / m in body frame */
        vec3_t drag_b = q_rotate_inv(s->q, v3_scale(s->vel, -s->drag_lin));
        f = v3_scale(v3_add(v3(0.0f, 0.0f, -thrust), drag_b), 1.0f / (s->mass * FM_GRAVITY));
    }
    *accel = v3(f.x + s->accel_noise * nrand(s), f.y + s->accel_noise * nrand(s),
                f.z + s->accel_noise * nrand(s));
}

SIM_API void sim_step(quad_sim_t *s, int n)
{
    const float dt = 1.0f / (float)s->cfg.gyro_rate_hz;
    const uint32_t dt_us = (uint32_t)(1e6f / (float)s->cfg.gyro_rate_hz + 0.5f);
    for (int k = 0; k < n; k++) {
        s->now_us += dt_us;
        s->t += dt;
        if (s->link_up && (uint32_t)(s->now_us - s->last_rc_us) >= s->rc_period_us) {
            fc_rc_update(&s->fc, &s->rc, s->now_us);
            s->last_rc_us = s->now_us;
        }
        vec3_t g, a;
        sense(s, &g, &a);
        if (fc_gyro_sample(&s->fc, g, a)) {
            fc_pid_step(&s->fc, s->now_us);
        }
        uint16_t d[4];
        float u[4];
        fc_motor_dshot(&s->fc, d);
        for (int i = 0; i < 4; i++) {
            u[i] = d[i] >= DSHOT_MIN_THROTTLE
                       ? (float)(d[i] - DSHOT_MIN_THROTTLE) / (float)DSHOT_RANGE
                       : 0.0f;
        }
        physics(s, u, dt);
    }
}

SIM_API void sim_run_seconds(quad_sim_t *s, float seconds)
{
    sim_step(s, (int)(seconds * (float)s->cfg.gyro_rate_hz + 0.5f));
}

SIM_API int sim_arm(quad_sim_t *s)
{
    sim_set_rc(s, 0, 0, 0, 0, 0, s->rc.angle_mode);
    sim_run_seconds(s, 1.5f);          /* gyro calibration */
    sim_set_rc(s, 0, 0, 0, 0, 1, s->rc.angle_mode);
    sim_run_seconds(s, 0.05f);
    return s->fc.armed ? 1 : 0;
}

SIM_API void sim_get_state(const quad_sim_t *s, float out[SIM_STATE_LEN])
{
    float r, p, y, er, ep, ey;
    q_to_euler(s->q, &r, &p, &y);
    q_to_euler(s->fc.q, &er, &ep, &ey);
    out[0] = s->t;
    out[1] = r * FM_RAD2DEG;
    out[2] = p * FM_RAD2DEG;
    out[3] = y * FM_RAD2DEG;
    out[4] = s->w.x * FM_RAD2DEG;
    out[5] = s->w.y * FM_RAD2DEG;
    out[6] = s->w.z * FM_RAD2DEG;
    out[7] = er * FM_RAD2DEG;
    out[8] = ep * FM_RAD2DEG;
    out[9] = ey * FM_RAD2DEG;
    out[10] = s->fc.rate_sp.x;
    out[11] = s->fc.rate_sp.y;
    out[12] = s->fc.rate_sp.z;
    for (int i = 0; i < 4; i++) out[13 + i] = s->fc.motor[i];
    out[17] = -s->pos.z;
    out[18] = s->fc.armed ? 1.0f : 0.0f;
    out[19] = (float)s->fc.fs;
}

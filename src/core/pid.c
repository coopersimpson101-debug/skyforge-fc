#include "pid.h"

void pid_axis_init(pid_axis_t *a, pid_gains_t g, float loop_hz,
                   float dterm_lpf_hz, float ff_lpf_hz)
{
    a->g = g;
    a->i_limit = 0.25f;
    a->out_limit = 1.0f;
    pt1_init(&a->dterm_lpf, dterm_lpf_hz, loop_hz);
    pt1_init(&a->ff_lpf, ff_lpf_hz, loop_hz);
    pid_axis_reset(a);
}

void pid_axis_reset(pid_axis_t *a)
{
    a->integ = 0.0f;
    a->prev_meas = 0.0f;
    a->prev_sp = 0.0f;
    a->dterm_lpf.y = 0.0f;
    a->ff_lpf.y = 0.0f;
    a->initialised = false;
    a->p = a->i = a->d = a->f = a->out = 0.0f;
}

float pid_axis_update(pid_axis_t *a, float sp, float meas, float dt,
                      float tpa, bool saturated)
{
    if (dt <= 0.0f) {
        return a->out;
    }
    if (!a->initialised) {
        /* avoid a derivative kick on the first sample */
        a->prev_meas = meas;
        a->prev_sp = sp;
        a->initialised = true;
    }
    const float err = sp - meas;

    a->p = a->g.kp * PID_P_SCALE * err * tpa;

    /* conditional integration: when saturated only allow I to unwind */
    const float di = a->g.ki * PID_I_SCALE * err * dt;
    if (!saturated || (a->integ * di) < 0.0f) {
        a->integ += di;
    }
    a->integ = fm_clampf(a->integ, -a->i_limit, a->i_limit);
    a->i = a->integ;

    /* derivative on measurement: no kick on setpoint steps */
    const float dmeas = (meas - a->prev_meas) / dt;
    a->d = -a->g.kd * PID_D_SCALE * pt1_apply(&a->dterm_lpf, dmeas) * tpa;

    const float dsp = (sp - a->prev_sp) / dt;
    a->f = a->g.kf * PID_F_SCALE * pt1_apply(&a->ff_lpf, dsp);

    a->prev_meas = meas;
    a->prev_sp = sp;
    a->out = fm_clampf(a->p + a->i + a->d + a->f, -a->out_limit, a->out_limit);
    return a->out;
}

float pid_tpa_factor(float throttle, float breakpoint, float rate)
{
    if (throttle <= breakpoint || breakpoint >= 1.0f) {
        return 1.0f;
    }
    float t = (throttle - breakpoint) / (1.0f - breakpoint);
    return 1.0f - fm_clampf(rate, 0.0f, 1.0f) * fm_clampf(t, 0.0f, 1.0f);
}

float rc_rate_curve(float stick, float center_dps, float max_dps, float expo)
{
    stick = fm_clampf(stick, -1.0f, 1.0f);
    expo = fm_clampf(expo, 0.0f, 1.0f);
    const float ax = fm_absf(stick);
    const float x4 = ax * ax * ax * ax;
    /* shape(1) = 1, shape'(0) = 0: blend of x|x| and x^5|x|-ish */
    const float shape = stick * ax * ((1.0f - expo) + expo * x4);
    const float extra = fm_maxf(0.0f, max_dps - center_dps);
    return stick * center_dps + extra * shape;
}

vec3_t angle_controller(quat_t q, float roll_sp_rad, float pitch_sp_rad,
                        float kp, float max_rate_dps)
{
    float yaw;
    q_to_euler(q, NULL, NULL, &yaw);
    const quat_t q_des = q_from_euler(roll_sp_rad, pitch_sp_rad, yaw);
    const quat_t q_err = q_mul(q_conj(q), q_des);
    const vec3_t rv = q_to_rotvec(q_err); /* body frame, shortest path */
    vec3_t r = v3_scale(rv, kp * FM_RAD2DEG);
    r.x = fm_clampf(r.x, -max_rate_dps, max_rate_dps);
    r.y = fm_clampf(r.y, -max_rate_dps, max_rate_dps);
    r.z = 0.0f; /* yaw is rate-controlled from the stick in angle mode */
    return r;
}

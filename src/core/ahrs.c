#include "ahrs.h"

void ahrs_init(ahrs_t *a, ahrs_type_e type, float madgwick_beta)
{
    a->type = type;
    madgwick_init(&a->mw, madgwick_beta);
    a->mw.boot_time_s = 2.0f;
    ekf_att_init(&a->ekf);
    a->aligned = false;
    a->acc_rate_gate = 1.0f; /* ~57 deg/s */
}

void ahrs_update(ahrs_t *a, vec3_t gyro_rads, vec3_t accel_g, float dt)
{
    if (!a->aligned) {
        float an = v3_norm(accel_g);
        if (an > 0.8f && an < 1.2f) {
            madgwick_align(&a->mw, accel_g);
            ekf_att_align(&a->ekf, accel_g);
            a->aligned = true;
        }
    }
    if (a->acc_rate_gate > 0.0f && v3_norm(gyro_rads) > a->acc_rate_gate) {
        accel_g = v3(0.0f, 0.0f, 0.0f); /* invalid -> gyro-only fallback */
    }
    if (a->type == AHRS_EKF) {
        ekf_att_step(&a->ekf, gyro_rads, accel_g, dt);
    } else {
        madgwick_update(&a->mw, gyro_rads, accel_g, dt);
    }
}

quat_t ahrs_quat(const ahrs_t *a)
{
    return a->type == AHRS_EKF ? a->ekf.q : a->mw.q;
}

vec3_t ahrs_bias(const ahrs_t *a)
{
    return a->type == AHRS_EKF ? a->ekf.bias : v3(0.0f, 0.0f, 0.0f);
}

bool ahrs_used_accel(const ahrs_t *a)
{
    return a->type == AHRS_EKF ? a->ekf.last_used_acc : a->mw.last_used_acc;
}

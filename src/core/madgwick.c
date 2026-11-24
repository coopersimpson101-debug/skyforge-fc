/*
 * madgwick.c - Madgwick AHRS, derived by hand for the NED/FRD convention.
 *
 * Objective  f(q) = R(q)^T e3 - v, with
 *   R^T e3 = [ 2(xz - wy),  2(yz + wx),  1 - 2(x^2 + y^2) ]
 * Jacobian (columns d/dw, d/dx, d/dy, d/dz):
 *   J = [ -2y   2z  -2w   2x ]
 *       [  2x   2w   2z   2y ]
 *       [   0  -4x  -4y    0 ]
 * Gradient step direction  g = J^T f, normalised.
 */
#include "madgwick.h"

void madgwick_init(madgwick_t *m, float beta)
{
    m->q = q_identity();
    m->beta = beta;
    m->beta_boot = 2.5f;
    m->boot_time_s = 0.0f; /* disabled unless caller sets it */
    m->t = 0.0f;
    m->acc_reject = 0.25f;
    m->last_used_acc = false;
}

void madgwick_align(madgwick_t *m, vec3_t accel_g)
{
    vec3_t d = v3_scale(accel_g, -1.0f); /* down in body */
    if (v3_norm(d) < 0.1f) {
        return;
    }
    d = v3_normalize(d);
    /* down_b = [-sin(pitch), sin(roll)cos(pitch), cos(roll)cos(pitch)] */
    float roll = fm_atan2f(d.y, d.z);
    float pitch = fm_asinf(-d.x);
    m->q = q_from_euler(roll, pitch, 0.0f);
}

void madgwick_update(madgwick_t *m, vec3_t gyro, vec3_t accel_g, float dt)
{
    quat_t q = m->q;
    /* rate of change from gyro: 0.5 * q (x) [0, w] */
    quat_t qdot;
    qdot.w = 0.5f * (-q.x * gyro.x - q.y * gyro.y - q.z * gyro.z);
    qdot.x = 0.5f * ( q.w * gyro.x + q.y * gyro.z - q.z * gyro.y);
    qdot.y = 0.5f * ( q.w * gyro.y - q.x * gyro.z + q.z * gyro.x);
    qdot.z = 0.5f * ( q.w * gyro.z + q.x * gyro.y - q.y * gyro.x);

    float an = v3_norm(accel_g);
    m->last_used_acc = false;
    if (an > 1e-3f && fm_absf(an - 1.0f) < m->acc_reject) {
        const vec3_t v = v3_scale(accel_g, -1.0f / an);
        const float f1 = 2.0f * (q.x * q.z - q.w * q.y) - v.x;
        const float f2 = 2.0f * (q.y * q.z + q.w * q.x) - v.y;
        const float f3 = 1.0f - 2.0f * (q.x * q.x + q.y * q.y) - v.z;
        /* g = J^T f */
        float gw = -2.0f * q.y * f1 + 2.0f * q.x * f2;
        float gx =  2.0f * q.z * f1 + 2.0f * q.w * f2 - 4.0f * q.x * f3;
        float gy = -2.0f * q.w * f1 + 2.0f * q.z * f2 - 4.0f * q.y * f3;
        float gz =  2.0f * q.x * f1 + 2.0f * q.y * f2;
        float gn = fm_sqrtf(gw * gw + gx * gx + gy * gy + gz * gz);
        if (gn > 1e-9f) {
            float beta = (m->t < m->boot_time_s) ? m->beta_boot : m->beta;
            float s = beta / gn;
            qdot.w -= s * gw;
            qdot.x -= s * gx;
            qdot.y -= s * gy;
            qdot.z -= s * gz;
            m->last_used_acc = true;
        }
    }

    q.w += qdot.w * dt;
    q.x += qdot.x * dt;
    q.y += qdot.y * dt;
    q.z += qdot.z * dt;
    m->q = q_normalize(q);
    m->t += dt;
}

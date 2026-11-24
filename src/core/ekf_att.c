/*
 * ekf_att.c - multiplicative error-state EKF (see ekf_att.h for the math).
 * Small fixed-size matrix algebra is written out by hand (no libraries).
 */
#include "ekf_att.h"

static void skew(vec3_t v, float m[3][3])
{
    m[0][0] = 0.0f;  m[0][1] = -v.z;  m[0][2] = v.y;
    m[1][0] = v.z;   m[1][1] = 0.0f;  m[1][2] = -v.x;
    m[2][0] = -v.y;  m[2][1] = v.x;   m[2][2] = 0.0f;
}

static bool inv3(const float a[3][3], float r[3][3])
{
    const float c00 = a[1][1] * a[2][2] - a[1][2] * a[2][1];
    const float c01 = a[1][2] * a[2][0] - a[1][0] * a[2][2];
    const float c02 = a[1][0] * a[2][1] - a[1][1] * a[2][0];
    const float det = a[0][0] * c00 + a[0][1] * c01 + a[0][2] * c02;
    if (fm_absf(det) < 1e-20f) {
        return false;
    }
    const float id = 1.0f / det;
    r[0][0] = c00 * id;
    r[0][1] = (a[0][2] * a[2][1] - a[0][1] * a[2][2]) * id;
    r[0][2] = (a[0][1] * a[1][2] - a[0][2] * a[1][1]) * id;
    r[1][0] = c01 * id;
    r[1][1] = (a[0][0] * a[2][2] - a[0][2] * a[2][0]) * id;
    r[1][2] = (a[0][2] * a[1][0] - a[0][0] * a[1][2]) * id;
    r[2][0] = c02 * id;
    r[2][1] = (a[0][1] * a[2][0] - a[0][0] * a[2][1]) * id;
    r[2][2] = (a[0][0] * a[1][1] - a[0][1] * a[1][0]) * id;
    return true;
}

static void symmetrize(float P[EKF_N][EKF_N])
{
    for (int i = 0; i < EKF_N; i++) {
        for (int j = i + 1; j < EKF_N; j++) {
            float s = 0.5f * (P[i][j] + P[j][i]);
            P[i][j] = s;
            P[j][i] = s;
        }
    }
}

void ekf_att_init(ekf_att_t *e)
{
    e->q = q_identity();
    e->bias = v3(0.0f, 0.0f, 0.0f);
    for (int i = 0; i < EKF_N; i++) {
        for (int j = 0; j < EKF_N; j++) {
            e->P[i][j] = 0.0f;
        }
    }
    for (int i = 0; i < 3; i++) {
        e->P[i][i] = 0.3f;              /* ~31 deg 1-sigma */
        e->P[i + 3][i + 3] = 2.5e-3f;   /* ~2.9 deg/s 1-sigma */
    }
    e->gyro_noise = 0.002f;
    e->bias_walk = 1e-4f;
    e->accel_noise = 0.5f;
    e->acc_reject = 0.3f;
    e->p_max_angle = 0.05f; /* keeps unobservable yaw variance from leaking
                             * into roll/pitch through [w]x at high rates */
    e->bias_max = 0.35f; /* 20 deg/s */
    e->gate_chi2 = 16.0f; /* ~99.9 % for 3 dof */
    e->gate_max_rejects = 2000;
    e->rejects = 0;
    e->total_rejects = 0;
    e->last_innov = 0.0f;
    e->last_used_acc = false;
}

void ekf_att_align(ekf_att_t *e, vec3_t accel_g)
{
    vec3_t d = v3_scale(accel_g, -1.0f);
    if (v3_norm(d) < 0.1f) {
        return;
    }
    d = v3_normalize(d);
    e->q = q_from_euler(fm_atan2f(d.y, d.z), fm_asinf(-d.x), 0.0f);
    for (int i = 0; i < 3; i++) {
        e->P[i][i] = 0.01f;
    }
}

void ekf_att_predict(ekf_att_t *e, vec3_t gyro, float dt)
{
    const vec3_t w = v3_sub(gyro, e->bias);
    e->q = q_normalize(q_mul(e->q, q_from_rotvec(v3_scale(w, dt))));

    /* Phi = [[I - [w]x dt, -I dt], [0, I]] */
    float Phi[EKF_N][EKF_N] = {{0}};
    float W[3][3];
    skew(w, W);
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            Phi[i][j] = (i == j ? 1.0f : 0.0f) - W[i][j] * dt;
        }
        Phi[i][i + 3] = -dt;
        Phi[i + 3][i + 3] = 1.0f;
    }

    float A[EKF_N][EKF_N];
    for (int i = 0; i < EKF_N; i++) {
        for (int j = 0; j < EKF_N; j++) {
            float s = 0.0f;
            for (int k = 0; k < EKF_N; k++) {
                s += Phi[i][k] * e->P[k][j];
            }
            A[i][j] = s;
        }
    }
    for (int i = 0; i < EKF_N; i++) {
        for (int j = 0; j < EKF_N; j++) {
            float s = 0.0f;
            for (int k = 0; k < EKF_N; k++) {
                s += A[i][k] * Phi[j][k];
            }
            e->P[i][j] = s;
        }
    }
    const float qa = e->gyro_noise * e->gyro_noise * dt;
    const float qb = e->bias_walk * e->bias_walk * dt;
    for (int i = 0; i < 3; i++) {
        e->P[i][i] += qa;
        e->P[i + 3][i + 3] += qb;
        if (e->P[i][i] > e->p_max_angle) {
            /* congruence scaling D P D keeps P positive semi-definite
             * (clamping only the diagonal would not) */
            const float s = fm_sqrtf(e->p_max_angle / e->P[i][i]);
            for (int k = 0; k < EKF_N; k++) {
                e->P[i][k] *= s;
                e->P[k][i] *= s;
            }
        }
    }
    symmetrize(e->P);
}

void ekf_att_update_accel(ekf_att_t *e, vec3_t accel_g)
{
    e->last_used_acc = false;
    const float an = v3_norm(accel_g);
    if (an < 1e-3f || fm_absf(an - 1.0f) > e->acc_reject) {
        return;
    }
    const vec3_t z = v3_scale(accel_g, -1.0f / an);
    const vec3_t g = q_body_down(e->q);
    const vec3_t y = v3_sub(z, g);
    e->last_innov = v3_norm(y);

    /* measurement noise, inflated when |a| deviates from 1 g */
    const float dev = (an - 1.0f) / e->acc_reject;
    const float r = e->accel_noise * e->accel_noise * (1.0f + 10.0f * dev * dev);

    float G[3][3];
    skew(g, G); /* H = [G 0] */

    /* PHt = P H^T  (6x3) : PHt[i][j] = sum_k P[i][k] G[j][k] */
    float PHt[EKF_N][3];
    for (int i = 0; i < EKF_N; i++) {
        for (int j = 0; j < 3; j++) {
            float s = 0.0f;
            for (int k = 0; k < 3; k++) {
                s += e->P[i][k] * G[j][k];
            }
            PHt[i][j] = s;
        }
    }
    /* S = H P H^T + R */
    float S[3][3], Si[3][3];
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            float s = 0.0f;
            for (int k = 0; k < 3; k++) {
                s += G[i][k] * PHt[k][j];
            }
            S[i][j] = s + (i == j ? r : 0.0f);
        }
    }
    if (!inv3(S, Si)) {
        return;
    }
    /* innovation gate: during manoeuvres the accelerometer measures thrust
     * and drag rather than gravity; reject statistically implausible
     * updates, but never lock out the accelerometer permanently. */
    if (e->gate_chi2 > 0.0f) {
        const float yv[3] = {y.x, y.y, y.z};
        float d2 = 0.0f;
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                d2 += yv[i] * Si[i][j] * yv[j];
            }
        }
        if (d2 > e->gate_chi2 && e->rejects < e->gate_max_rejects) {
            e->rejects++;
            e->total_rejects++;
            return;
        }
    }
    e->rejects = 0;
    float K[EKF_N][3];
    for (int i = 0; i < EKF_N; i++) {
        for (int j = 0; j < 3; j++) {
            K[i][j] = PHt[i][0] * Si[0][j] + PHt[i][1] * Si[1][j] + PHt[i][2] * Si[2][j];
        }
    }
    float dx[EKF_N];
    for (int i = 0; i < EKF_N; i++) {
        dx[i] = K[i][0] * y.x + K[i][1] * y.y + K[i][2] * y.z;
    }

    /* Joseph form: P = (I-KH) P (I-KH)^T + r K K^T */
    float IKH[EKF_N][EKF_N];
    for (int i = 0; i < EKF_N; i++) {
        for (int j = 0; j < EKF_N; j++) {
            float kh = 0.0f;
            if (j < 3) {
                kh = K[i][0] * G[0][j] + K[i][1] * G[1][j] + K[i][2] * G[2][j];
            }
            IKH[i][j] = (i == j ? 1.0f : 0.0f) - kh;
        }
    }
    float A[EKF_N][EKF_N];
    for (int i = 0; i < EKF_N; i++) {
        for (int j = 0; j < EKF_N; j++) {
            float s = 0.0f;
            for (int k = 0; k < EKF_N; k++) {
                s += IKH[i][k] * e->P[k][j];
            }
            A[i][j] = s;
        }
    }
    for (int i = 0; i < EKF_N; i++) {
        for (int j = 0; j < EKF_N; j++) {
            float s = 0.0f;
            for (int k = 0; k < EKF_N; k++) {
                s += A[i][k] * IKH[j][k];
            }
            e->P[i][j] = s + r * (K[i][0] * K[j][0] + K[i][1] * K[j][1] + K[i][2] * K[j][2]);
        }
    }
    symmetrize(e->P);

    /* inject error state into nominal state */
    e->q = q_normalize(q_mul(e->q, q_from_rotvec(v3(dx[0], dx[1], dx[2]))));
    e->bias.x = fm_clampf(e->bias.x + dx[3], -e->bias_max, e->bias_max);
    e->bias.y = fm_clampf(e->bias.y + dx[4], -e->bias_max, e->bias_max);
    e->bias.z = fm_clampf(e->bias.z + dx[5], -e->bias_max, e->bias_max);
    e->last_used_acc = true;
}

void ekf_att_step(ekf_att_t *e, vec3_t gyro, vec3_t accel_g, float dt)
{
    ekf_att_predict(e, gyro, dt);
    ekf_att_update_accel(e, accel_g);
}

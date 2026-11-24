/*
 * Madgwick and EKF convergence on synthetic IMU data.
 * Truth is integrated independently; sensors = truth + bias + white noise.
 */
#include "../src/core/ahrs.h"
#include "tinytest.h"

static unsigned rng = 12345;
static float gauss(void)
{
    /* sum of 12 uniforms - 6 ~ N(0,1) (Irwin-Hall) */
    float s = 0;
    for (int i = 0; i < 12; i++) {
        rng = rng * 1664525u + 1013904223u;
        s += (float)(rng >> 8) / 16777216.0f;
    }
    return s - 6.0f;
}

typedef struct {
    quat_t q;
    vec3_t bias;
    float gyro_noise, acc_noise;
} truth_t;

static void sense(const truth_t *t, vec3_t w, vec3_t *g, vec3_t *a)
{
    *g = v3(w.x + t->bias.x + t->gyro_noise * gauss(), w.y + t->bias.y + t->gyro_noise * gauss(),
            w.z + t->bias.z + t->gyro_noise * gauss());
    vec3_t d = q_body_down(t->q); /* no linear acceleration: f = -g_down */
    *a = v3(-d.x + t->acc_noise * gauss(), -d.y + t->acc_noise * gauss(),
            -d.z + t->acc_noise * gauss());
}

static float tilt_err_deg(quat_t est, quat_t truth)
{
    /* angle between estimated and true gravity directions (yaw-free) */
    vec3_t a = q_body_down(est), b = q_body_down(truth);
    return fm_acosf(fm_clampf(v3_dot(a, b), -1, 1)) * FM_RAD2DEG;
}

static void test_madgwick_static_convergence(void)
{
    truth_t t = {q_from_euler(30 * FM_DEG2RAD, -20 * FM_DEG2RAD, 0), {0, 0, 0}, 0.005f, 0.02f};
    madgwick_t m;
    madgwick_init(&m, 0.1f); /* start at identity: 36 deg tilt error */
    const float dt = 1.0f / 1000.0f;
    float e0 = tilt_err_deg(m.q, t.q);
    for (int i = 0; i < 6000; i++) {
        vec3_t g, a;
        sense(&t, v3(0, 0, 0), &g, &a);
        madgwick_update(&m, g, a, dt);
    }
    float e = tilt_err_deg(m.q, t.q);
    printf("    madgwick: initial tilt error %.1f deg -> %.2f deg after 6 s\n", e0, e);
    CHECK(e0 > 30.0f);
    CHECK(e < 1.0f);
}

static void test_madgwick_tracks_rotation(void)
{
    /* coning-ish motion: rates on all axes, gyro+accel consistent */
    truth_t t = {q_identity(), {0, 0, 0}, 0.003f, 0.02f};
    madgwick_t m;
    madgwick_init(&m, 0.05f);
    const float dt = 1.0f / 2000.0f;
    float max_err = 0;
    for (int i = 0; i < 20000; i++) {
        float time = i * dt;
        vec3_t w = v3(1.5f * sinf(2.0f * time), 1.2f * cosf(1.3f * time), 0.8f);
        vec3_t g, a;
        sense(&t, w, &g, &a);
        madgwick_update(&m, g, a, dt);
        t.q = q_normalize(q_mul(t.q, q_from_rotvec(v3_scale(w, dt))));
        if (i > 2000) max_err = fmaxf(max_err, tilt_err_deg(m.q, t.q));
    }
    printf("    madgwick: max tilt error while rotating = %.2f deg\n", max_err);
    CHECK(max_err < 2.0f);
}

static void test_madgwick_gyro_only_fallback(void)
{
    madgwick_t m;
    madgwick_init(&m, 0.1f);
    /* accel reads 2.5 g (hard manoeuvre) -> must be ignored */
    for (int i = 0; i < 1000; i++) {
        madgwick_update(&m, v3(FM_HALF_PI, 0, 0), v3(0, 0, -2.5f), 0.001f);
        CHECK(!m.last_used_acc);
    }
    float r;
    q_to_euler(m.q, &r, NULL, NULL);
    CHECK_NEAR(r, FM_HALF_PI, 1e-3); /* exact gyro integration: 90 deg */
    /* zero accel (sensor failure) also falls back */
    madgwick_update(&m, v3(0, 0, 0), v3(0, 0, 0), 0.001f);
    CHECK(!m.last_used_acc);
}

static void test_ekf_static_convergence_and_bias(void)
{
    truth_t t = {q_from_euler(-25 * FM_DEG2RAD, 15 * FM_DEG2RAD, 0),
                 {0.03f, -0.02f, 0.0f}, 0.005f, 0.02f};
    ekf_att_t e;
    ekf_att_init(&e);
    e.gyro_noise = 0.01f;   /* faster tuning for a bench-style test */
    e.accel_noise = 0.05f;
    e.bias_walk = 1e-3f;
    const float dt = 1.0f / 1000.0f;
    for (int i = 0; i < 60000; i++) {
        vec3_t g, a;
        sense(&t, v3(0, 0, 0), &g, &a);
        ekf_att_step(&e, g, a, dt);
    }
    float err = tilt_err_deg(e.q, t.q);
    /* Only the bias component perpendicular to gravity is observable from a
     * static pose (rotation about the gravity axis does not change the
     * accelerometer), so compare that projection. */
    vec3_t d = q_body_down(t.q);
    vec3_t be = v3_sub(e.bias, t.bias);
    vec3_t be_perp = v3_sub(be, v3_scale(d, v3_dot(be, d)));
    printf("    ekf: tilt error %.3f deg, bias est (%.4f, %.4f, %.4f) true (0.03, -0.02, 0), "
           "observable bias error %.5f rad/s\n",
           err, e.bias.x, e.bias.y, e.bias.z, v3_norm(be_perp));
    CHECK(err < 0.5f);
    CHECK(v3_norm(be_perp) < 0.002f);
    /* covariance stays symmetric positive */
    for (int i = 0; i < EKF_N; i++) {
        CHECK(e.P[i][i] > 0.0f);
        for (int j = 0; j < EKF_N; j++) CHECK_NEAR(e.P[i][j], e.P[j][i], 1e-9);
    }
}

static void test_ekf_bias_drift_vs_madgwick(void)
{
    /* With an uncompensated gyro bias, Madgwick settles to a constant tilt
     * error (beta has to fight the bias) while the EKF learns the bias.
     * Craft at level rest for 40 s with a 3.7 deg/s x/y gyro bias. */
    truth_t t = {q_identity(), {0.05f, 0.04f, 0.0f}, 0.003f, 0.02f};
    ekf_att_t e;
    ekf_att_init(&e);
    e.gyro_noise = 0.01f;
    e.accel_noise = 0.05f;
    e.bias_walk = 1e-3f;
    madgwick_t m;
    madgwick_init(&m, 0.02f);
    const float dt = 1.0f / 1000.0f;
    for (int i = 0; i < 40000; i++) {
        vec3_t g, a;
        sense(&t, v3(0, 0, 0), &g, &a);
        ekf_att_step(&e, g, a, dt);
        madgwick_update(&m, g, a, dt);
    }
    float ee = tilt_err_deg(e.q, t.q), em = tilt_err_deg(m.q, t.q);
    printf("    with 3.7 deg/s bias: EKF tilt error %.2f deg, Madgwick %.2f deg\n", ee, em);
    CHECK(ee < 0.5f);
    CHECK(ee < em);
}

static void test_ahrs_front_end(void)
{
    ahrs_t a;
    ahrs_init(&a, AHRS_EKF, 0.05f);
    vec3_t acc = v3(0.5f, 0.0f, -0.8660254f); /* pitched up 30 deg */
    ahrs_update(&a, v3(0, 0, 0), acc, 0.001f);
    CHECK(a.aligned);
    float p;
    q_to_euler(ahrs_quat(&a), NULL, &p, NULL);
    CHECK_NEAR(p * FM_RAD2DEG, 30.0, 0.5);
    /* high rotation rate gates the accelerometer */
    ahrs_update(&a, v3(3.0f, 0, 0), acc, 0.001f);
    CHECK(!ahrs_used_accel(&a));
}

int main(void)
{
    RUN(test_madgwick_static_convergence);
    RUN(test_madgwick_tracks_rotation);
    RUN(test_madgwick_gyro_only_fallback);
    RUN(test_ekf_static_convergence_and_bias);
    RUN(test_ekf_bias_drift_vs_madgwick);
    RUN(test_ahrs_front_end);
    return TT_SUMMARY();
}

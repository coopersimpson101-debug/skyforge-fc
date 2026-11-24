/*
 * PID, rate curve, angle controller and mixer tests.
 */
#include "../src/core/mixer.h"
#include "../src/core/pid.h"
#include "tinytest.h"

static pid_axis_t make_pid(float p, float i, float d, float f)
{
    pid_axis_t a;
    pid_gains_t g = {p, i, d, f};
    pid_axis_init(&a, g, 1000.0f, 0.0f, 0.0f); /* filters off */
    return a;
}

static void test_pid_terms(void)
{
    pid_axis_t a = make_pid(50, 0, 0, 0);
    float out = pid_axis_update(&a, 100, 0, 0.001f, 1.0f, false);
    CHECK_NEAR(out, 50 * PID_P_SCALE * 100, 1e-7);
    /* TPA halves P */
    out = pid_axis_update(&a, 100, 0, 0.001f, 0.5f, false);
    CHECK_NEAR(out, 0.5 * 50 * PID_P_SCALE * 100, 1e-7);

    /* I integrates error*dt */
    a = make_pid(0, 80, 0, 0);
    for (int k = 0; k < 100; k++) pid_axis_update(&a, 10, 0, 0.001f, 1, false);
    CHECK_NEAR(a.integ, 80 * PID_I_SCALE * 10 * 0.1, 1e-6);
}

static void test_pid_d_on_measurement_no_kick(void)
{
    pid_axis_t a = make_pid(0, 0, 40, 0);
    pid_axis_update(&a, 0, 0, 0.001f, 1, false);
    /* setpoint step: no derivative kick */
    float out = pid_axis_update(&a, 500, 0, 0.001f, 1, false);
    CHECK_NEAR(out, 0, 1e-9);
    /* measurement moving +1 deg/s per ms -> D opposes it */
    out = pid_axis_update(&a, 500, 1, 0.001f, 1, false);
    CHECK_NEAR(out, -40 * PID_D_SCALE * 1000.0, 1e-7);
}

static void test_pid_feedforward(void)
{
    pid_axis_t a = make_pid(0, 0, 0, 100);
    pid_axis_update(&a, 0, 0, 0.001f, 1, false);
    float out = pid_axis_update(&a, 10, 0, 0.001f, 1, false); /* 10000 dps/s */
    CHECK_NEAR(out, 100 * PID_F_SCALE * 10000.0, 1e-6);
    out = pid_axis_update(&a, 10, 0, 0.001f, 1, false);       /* constant sp */
    CHECK_NEAR(out, 0, 1e-9);
}

static void test_pid_anti_windup(void)
{
    pid_axis_t a = make_pid(0, 100, 0, 0);
    a.i_limit = 0.2f;
    for (int k = 0; k < 100000; k++) pid_axis_update(&a, 500, 0, 0.001f, 1, false);
    CHECK_NEAR(a.integ, 0.2, 1e-6); /* clamped */
    /* saturated: I must not grow further in the same direction ... */
    pid_axis_reset(&a);
    for (int k = 0; k < 100; k++) pid_axis_update(&a, 100, 0, 0.001f, 1, true);
    CHECK_NEAR(a.integ, 0, 1e-9);
    /* ... but may unwind */
    a.integ = 0.1f;
    pid_axis_update(&a, -100, 0, 0.001f, 1, true);
    CHECK(a.integ < 0.1f);
    CHECK_NEAR(pid_tpa_factor(0.5f, 0.65f, 0.65f), 1.0, 1e-6);
    CHECK_NEAR(pid_tpa_factor(1.0f, 0.65f, 0.65f), 0.35, 1e-6);
}

static void test_pid_step_response_on_plant(void)
{
    /* rate plant with first-order motor lag:  J w' = k * m,  m' = (u - m)/tau */
    pid_axis_t a;
    pid_gains_t g = {55, 50, 45, 0};
    pid_axis_init(&a, g, 4000.0f, 100.0f, 0.0f);
    const float dt = 1.0f / 4000.0f, k_over_j = 60000.0f, tau = 0.02f;
    float w = 0, m = 0, peak = 0, t90 = -1;
    for (int i = 0; i < 4000; i++) {
        float u = pid_axis_update(&a, 300.0f, w, dt, 1, false);
        m += (u - m) * dt / tau;
        w += k_over_j * m * dt; /* deg/s */
        peak = fmaxf(peak, w);
        if (t90 < 0 && w > 270.0f) t90 = i * dt;
    }
    printf("    plant step 300 dps: t90 %.1f ms, overshoot %.1f %%, final %.1f dps\n",
           t90 * 1000, 100 * (peak - 300) / 300, w);
    CHECK(t90 > 0 && t90 < 0.1f);
    CHECK(peak < 1.25f * 300.0f);
    CHECK_NEAR(w, 300.0, 3.0);  /* integral removes steady-state error */
}

static void test_rate_curve(void)
{
    CHECK_NEAR(rc_rate_curve(0, 200, 670, 0.54f), 0, 1e-6);
    CHECK_NEAR(rc_rate_curve(1, 200, 670, 0.54f), 670, 1e-3);
    CHECK_NEAR(rc_rate_curve(-1, 200, 670, 0.54f), -670, 1e-3);
    /* slope at centre equals the centre sensitivity */
    CHECK_NEAR(rc_rate_curve(0.001f, 200, 670, 0.54f) / 0.001f, 200, 1.0);
    /* monotonic */
    float prev = -1e9f;
    for (int i = -100; i <= 100; i++) {
        float r = rc_rate_curve(i / 100.0f, 200, 670, 0.54f);
        CHECK(r >= prev);
        prev = r;
    }
}

static void test_angle_controller(void)
{
    /* level, ask 10 deg right roll -> positive roll rate kp*10 */
    vec3_t r = angle_controller(q_identity(), 10 * FM_DEG2RAD, 0, 5.0f, 500);
    CHECK_NEAR(r.x, 50.0, 0.1);
    CHECK_NEAR(r.y, 0.0, 1e-3);
    /* rolled 20 deg, ask level -> negative roll rate */
    r = angle_controller(q_from_euler(20 * FM_DEG2RAD, 0, 1.0f), 0, 0, 5.0f, 500);
    CHECK_NEAR(r.x, -100.0, 0.2);
    /* pitch 30 deg nose up, ask level -> negative pitch rate, with limit */
    r = angle_controller(q_from_euler(0, 30 * FM_DEG2RAD, -2.0f), 0, 0, 30.0f, 400);
    CHECK_NEAR(r.y, -400.0, 1e-3);
    /* nose-down request -> negative pitch rate */
    r = angle_controller(q_identity(), 0, -10 * FM_DEG2RAD, 5.0f, 500);
    CHECK_NEAR(r.y, -50.0, 0.1);
    /* errors are body-frame: yawed 90 deg, a roll request is still roll */
    r = angle_controller(q_from_euler(0, 0, FM_HALF_PI), 10 * FM_DEG2RAD, 0, 5.0f, 500);
    CHECK_NEAR(r.x, 50.0, 0.1);
    CHECK_NEAR(r.y, 0.0, 0.1);
}

static void test_mixer_basic(void)
{
    mixer_t m;
    float out[4];
    mixer_init_quadx(&m, false, false);
    CHECK(!mixer_mix(&m, 0.5f, 0, 0, 0, out));
    for (int i = 0; i < 4; i++) CHECK_NEAR(out[i], 0.5, 1e-6);
    /* roll right: left motors (3,4) up */
    mixer_mix(&m, 0.5f, 0.1f, 0, 0, out);
    CHECK(out[2] > 0.5f && out[3] > 0.5f && out[0] < 0.5f && out[1] < 0.5f);
    /* pitch up: front motors (2,4) up */
    mixer_mix(&m, 0.5f, 0, 0.1f, 0, out);
    CHECK(out[1] > 0.5f && out[3] > 0.5f && out[0] < 0.5f && out[2] < 0.5f);
    /* yaw right: CCW motors (2,3) up; reversed props flip it */
    mixer_mix(&m, 0.5f, 0, 0, 0.1f, out);
    CHECK(out[1] > 0.5f && out[2] > 0.5f);
    mixer_init_quadx(&m, false, true);
    mixer_mix(&m, 0.5f, 0, 0, 0.1f, out);
    CHECK(out[0] > 0.5f && out[3] > 0.5f);
}

static void test_mixer_saturation_airmode(void)
{
    mixer_t m;
    float out[4];
    mixer_init_quadx(&m, true, false);
    /* zero throttle, roll demand: airmode raises throttle so both sides act */
    CHECK(mixer_mix(&m, 0.0f, 0.2f, 0, 0, out));
    CHECK_NEAR(out[0], 0.0, 1e-6);
    CHECK_NEAR(out[2], 0.2, 1e-6); /* full differential preserved */
    /* huge demand: rpy scaled to fit, spread exactly 1, direction kept */
    CHECK(mixer_mix(&m, 0.5f, 2.0f, 1.0f, 0, out));
    float mn = 1, mx = 0;
    for (int i = 0; i < 4; i++) {
        mn = fminf(mn, out[i]);
        mx = fmaxf(mx, out[i]);
        CHECK(out[i] >= 0.0f && out[i] <= 1.0f);
    }
    CHECK_NEAR(mx - mn, 1.0, 1e-5);
    CHECK_NEAR(m.rpy_scale, 1.0 / 3.0, 1e-5); /* spread was 3 */
    /* full throttle: airmode shifts down to keep differential */
    mixer_mix(&m, 1.0f, 0.1f, 0, 0, out);
    CHECK_NEAR(out[2] - out[0], 0.1, 1e-5);
    /* without airmode the same command is clipped */
    mixer_init_quadx(&m, false, false);
    CHECK(mixer_mix(&m, 0.0f, 0.2f, 0, 0, out));
    CHECK_NEAR(out[0], 0.0, 1e-6);
    CHECK_NEAR(out[2], 0.1, 1e-6); /* half the authority */
}

int main(void)
{
    RUN(test_pid_terms);
    RUN(test_pid_d_on_measurement_no_kick);
    RUN(test_pid_feedforward);
    RUN(test_pid_anti_windup);
    RUN(test_pid_step_response_on_plant);
    RUN(test_rate_curve);
    RUN(test_angle_controller);
    RUN(test_mixer_basic);
    RUN(test_mixer_saturation_airmode);
    return TT_SUMMARY();
}

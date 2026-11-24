/*
 * Closed-loop tests: the real FC core flying the rigid-body simulator.
 */
#include "../sim/quad_sim.h"
#include "tinytest.h"

static float st[SIM_STATE_LEN];

static void takeoff(quad_sim_t *s, int angle_mode)
{
    sim_set_rc(s, 0, 0, 0, 0, 0, angle_mode);
    CHECK(sim_arm(s));
    float hov = sim_hover_throttle(s);
    sim_set_rc(s, 0, 0, 0, hov + 0.08f, 1, angle_mode);
    sim_run_seconds(s, 1.0f);
    sim_set_rc(s, 0, 0, 0, hov, 1, angle_mode);
    sim_run_seconds(s, 1.0f);
    sim_get_state(s, st);
    CHECK(st[17] > 0.3f); /* airborne */
}

static void angle_recovery(int ahrs_type)
{
    quad_sim_t *s = sim_create();
    sim_set_param(s, "ahrs_type", (float)ahrs_type);
    takeoff(s, 1);
    /* gust: 0.6 N m roll + 0.25 N m pitch for 150 ms */
    sim_set_disturbance(s, 0.6f, -0.25f, 0.0f);
    float peak = 0;
    for (int i = 0; i < 150; i++) {
        sim_run_seconds(s, 0.001f);
        sim_get_state(s, st);
        peak = fmaxf(peak, fabsf(st[1]));
    }
    sim_set_disturbance(s, 0, 0, 0);
    for (int i = 0; i < 300; i++) { /* peak may come after the gust ends */
        sim_run_seconds(s, 0.001f);
        sim_get_state(s, st);
        peak = fmaxf(peak, fabsf(st[1]));
    }
    sim_run_seconds(s, 0.7f); /* 1.0 s after the gust */
    sim_get_state(s, st);
    printf("    ahrs=%d peak roll %.1f deg -> after 1 s: roll %.2f pitch %.2f, p %.1f dps\n",
           ahrs_type, peak, st[1], st[2], st[4]);
    CHECK(peak > 15.0f);            /* it was genuinely disturbed */
    CHECK(fabsf(st[1]) < 3.0f);
    CHECK(fabsf(st[2]) < 3.0f);
    CHECK(fabsf(st[4]) < 20.0f);
    CHECK(st[18] > 0.5f);           /* still armed */
    sim_destroy(s);
}

static void test_angle_mode_recovers_from_gust_madgwick(void) { angle_recovery(0); }
static void test_angle_mode_recovers_from_gust_ekf(void) { angle_recovery(1); }

static void test_angle_mode_tracks_stick(void)
{
    quad_sim_t *s = sim_create();
    takeoff(s, 1);
    float hov = sim_hover_throttle(s);
    /* half right roll stick -> 0.5 * 55 = 27.5 deg bank */
    sim_set_rc(s, 0.5f, 0, 0, hov, 1, 1);
    sim_run_seconds(s, 0.6f);
    sim_get_state(s, st);
    printf("    roll setpoint 27.5 deg -> %.2f deg after 0.6 s\n", st[1]);
    CHECK_NEAR(st[1], 27.5, 2.5);
    /* forward pitch stick = nose down */
    sim_set_rc(s, 0, 0.4f, 0, hov, 1, 1);
    sim_run_seconds(s, 0.6f);
    sim_get_state(s, st);
    CHECK_NEAR(st[2], -22.0, 2.5);
    sim_destroy(s);
}

static void test_acro_rate_step_response(void)
{
    quad_sim_t *s = sim_create();
    takeoff(s, 0);
    float hov = sim_hover_throttle(s);
    sim_set_rc(s, 0.6f, 0, 0, hov, 1, 0);
    sim_step(s, 1);
    sim_get_state(s, st);
    /* wait for the RC frame carrying the step */
    for (int i = 0; i < 100 && st[10] < 1.0f; i++) {
        sim_step(s, 1);
        sim_get_state(s, st);
    }
    const float t0 = st[0];
    float sp = 0, peak = 0, t90 = -1, err_sum = 0;
    int err_n = 0;
    float max_est_err = 0;
    while (st[0] - t0 < 0.35f) {
        sim_step(s, 1);
        sim_get_state(s, st);
        sp = st[10];
        peak = fmaxf(peak, st[4]);
        if (t90 < 0 && st[4] >= 0.9f * sp) t90 = st[0] - t0;
        if (st[0] - t0 > 0.15f) {
            err_sum += fabsf(st[4] - sp);
            err_n++;
        }
        max_est_err = fmaxf(max_est_err, fabsf(st[1] - st[7]));
    }
    float overshoot = 100.0f * (peak - sp) / sp;
    printf("    roll rate step to %.0f dps: t90 = %.1f ms, overshoot = %.1f %%, "
           "mean |err| after 150 ms = %.1f dps, max attitude est. error %.2f deg\n",
           sp, t90 * 1000.0f, overshoot, err_sum / err_n, max_est_err);
    CHECK(t90 > 0 && t90 < 0.08f);
    CHECK(overshoot < 20.0f);
    CHECK(err_sum / err_n < 0.08f * sp);
    CHECK(max_est_err < 3.0f);
    sim_destroy(s);
}

static void test_failsafe_lands_and_disarms(void)
{
    quad_sim_t *s = sim_create();
    sim_set_param(s, "fs_land_time_s", 3.0f);
    takeoff(s, 0);
    sim_set_link(s, 0);
    sim_run_seconds(s, 0.5f);
    sim_get_state(s, st);
    CHECK_EQ_INT((int)st[19], FS_LANDING);
    CHECK(st[18] > 0.5f);
    float max_tilt = 0;
    for (int i = 0; i < 20; i++) {
        sim_run_seconds(s, 0.1f);
        sim_get_state(s, st);
        max_tilt = fmaxf(max_tilt, fmaxf(fabsf(st[1]), fabsf(st[2])));
    }
    CHECK(max_tilt < 5.0f);         /* levelled while landing */
    sim_run_seconds(s, 1.0f);
    sim_get_state(s, st);
    CHECK(st[18] < 0.5f);           /* disarmed after fs_land_time */
    CHECK_EQ_INT((int)st[19], FS_DISARMED);
    /* link back with switch still ON: must not re-arm */
    sim_set_link(s, 1);
    sim_run_seconds(s, 0.5f);
    sim_get_state(s, st);
    CHECK(st[18] < 0.5f);
    sim_destroy(s);
}

static void test_airmode_full_authority_at_zero_throttle(void)
{
    quad_sim_t *s = sim_create();
    takeoff(s, 0);
    /* throttle cut + full roll: airmode must still produce the roll rate */
    sim_set_rc(s, 1.0f, 0, 0, 0.0f, 1, 0);
    sim_run_seconds(s, 0.25f);
    sim_get_state(s, st);
    printf("    zero throttle, full roll stick: p = %.0f dps (setpoint %.0f)\n", st[4], st[10]);
    CHECK(st[4] > 0.6f * st[10]);
    sim_destroy(s);
}

int main(void)
{
    RUN(test_angle_mode_recovers_from_gust_madgwick);
    RUN(test_angle_mode_recovers_from_gust_ekf);
    RUN(test_angle_mode_tracks_stick);
    RUN(test_acro_rate_step_response);
    RUN(test_failsafe_lands_and_disarms);
    RUN(test_airmode_full_authority_at_zero_throttle);
    return TT_SUMMARY();
}

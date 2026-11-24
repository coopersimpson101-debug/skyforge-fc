/*
 * sim_main.c - command line front-end for the closed-loop simulator.
 *
 *   sim_main <scenario> [param=value ...] [--csv]
 *
 * scenarios:
 *   step      acro roll-rate step (stick 0.6) from hover, 0.4 s
 *   gust      angle mode, 150 ms roll/pitch torque gust, recovery
 *   failsafe  link loss in hover -> level landing -> disarm
 *
 * Parameters are the same names as the firmware CLI ('dump' list), e.g.
 *   sim_main step p_roll=50 d_roll=40 f_roll=90 --csv > step.csv
 */
#include <stdio.h>
#include <string.h>

#include "quad_sim.h"

static void print_header(void)
{
    printf("t,roll,pitch,yaw,p,q,r,est_roll,est_pitch,est_yaw,sp_p,sp_q,sp_r,"
           "m1,m2,m3,m4,alt,armed,fs\n");
}

static void print_row(const quad_sim_t *s)
{
    float st[SIM_STATE_LEN];
    sim_get_state(s, st);
    for (int i = 0; i < SIM_STATE_LEN; i++) {
        printf(i ? ",%.5g" : "%.5g", st[i]);
    }
    printf("\n");
}

static void run_logged(quad_sim_t *s, float seconds, int csv, float *t0)
{
    int steps = (int)(seconds * 1000.0f + 0.5f);
    for (int i = 0; i < steps; i++) {
        sim_run_seconds(s, 0.001f);
        if (csv) {
            print_row(s);
        }
    }
    (void)t0;
}

int main(int argc, char **argv)
{
    if (argc < 2) {
        fprintf(stderr, "usage: %s step|gust|failsafe [param=value ...] [--csv]\n", argv[0]);
        return 2;
    }
    quad_sim_t *s = sim_create();
    int csv = 0;
    for (int i = 2; i < argc; i++) {
        if (strcmp(argv[i], "--csv") == 0) {
            csv = 1;
            continue;
        }
        char name[64];
        float v;
        const char *eq = strchr(argv[i], '=');
        if (!eq || eq - argv[i] >= (int)sizeof(name) || sscanf(eq + 1, "%f", &v) != 1) {
            fprintf(stderr, "bad argument '%s'\n", argv[i]);
            return 2;
        }
        memcpy(name, argv[i], (size_t)(eq - argv[i]));
        name[eq - argv[i]] = 0;
        if (!sim_set_param(s, name, v)) {
            fprintf(stderr, "unknown/out of range parameter '%s'\n", name);
            return 2;
        }
    }
    const char *sc = argv[1];
    const int angle = strcmp(sc, "gust") == 0;
    float st[SIM_STATE_LEN];
    if (csv) print_header();

    sim_set_rc(s, 0, 0, 0, 0, 0, angle);
    if (!sim_arm(s)) {
        fprintf(stderr, "arming failed (flags 0x%x)\n", (unsigned)s->fc.arm_block);
        return 1;
    }
    const float hov = sim_hover_throttle(s);
    sim_set_rc(s, 0, 0, 0, hov + 0.08f, 1, angle);
    run_logged(s, 1.0f, csv, NULL);
    sim_set_rc(s, 0, 0, 0, hov, 1, angle);
    run_logged(s, 1.0f, csv, NULL);

    if (strcmp(sc, "step") == 0) {
        sim_set_rc(s, 0.6f, 0, 0, hov, 1, 0);
        float peak = 0, sp = 0, t90 = -1, t0 = s->t;
        for (int i = 0; i < 400; i++) {
            sim_run_seconds(s, 0.001f);
            sim_get_state(s, st);
            if (csv) print_row(s);
            if (st[10] > sp) sp = st[10];
            if (st[4] > peak) peak = st[4];
            if (t90 < 0 && sp > 1 && st[4] > 0.9f * sp) t90 = s->t - t0;
        }
        sim_set_rc(s, 0, 0, 0, hov, 1, 0);
        run_logged(s, 0.3f, csv, NULL);
        if (!csv) {
            printf("roll rate step: setpoint %.0f dps, t90 %.1f ms (incl. RC latency), "
                   "overshoot %.1f %%\n", sp, t90 * 1000, 100 * (peak - sp) / sp);
        }
    } else if (strcmp(sc, "gust") == 0) {
        sim_set_disturbance(s, 0.6f, -0.25f, 0.0f);
        float peak = 0;
        for (int i = 0; i < 150; i++) {
            sim_run_seconds(s, 0.001f);
            sim_get_state(s, st);
            if (csv) print_row(s);
            if (st[1] > peak) peak = st[1];
        }
        sim_set_disturbance(s, 0, 0, 0);
        for (int i = 0; i < 1500; i++) {
            sim_run_seconds(s, 0.001f);
            sim_get_state(s, st);
            if (csv) print_row(s);
            if (st[1] > peak) peak = st[1];
        }
        if (!csv) {
            printf("gust: peak roll %.1f deg, after 1.5 s roll %.2f pitch %.2f deg\n",
                   peak, st[1], st[2]);
        }
    } else if (strcmp(sc, "failsafe") == 0) {
        sim_set_link(s, 0);
        run_logged(s, 7.0f, csv, NULL);
        sim_get_state(s, st);
        if (!csv) {
            printf("failsafe: armed=%d fs_state=%d alt=%.2f m\n", (int)st[18], (int)st[19], st[17]);
        }
    } else {
        fprintf(stderr, "unknown scenario\n");
        return 2;
    }
    sim_destroy(s);
    return 0;
}

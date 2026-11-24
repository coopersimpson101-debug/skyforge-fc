/*
 * quad_sim.h - closed-loop rigid-body quadcopter simulator that runs the
 * REAL flight controller core (src/core/fc.c) in the loop.
 *
 * Physics (NED world, FRD body, explicit Euler at the gyro rate):
 *   motor speed      s_i' = (u_i - s_i) / tau          u_i from DShot value
 *   thrust           T_i  = T_max * s_i^2              along body -z
 *   reaction torque  Q_i  = -/+ k_q T_i                (CW / CCW prop)
 *   torque           tau  = sum r_i x F_i + Q + tau_dist - c_w w
 *   rotation         I w' = tau - w x (I w)
 *   translation      m a  = R F_b + m g e3 - c_v v
 * Sensors: gyro = w + bias + noise + vibration, accel = R^T (a - g e3)/g + noise.
 *
 * The API is plain C functions on an opaque handle so Python can drive
 * it through ctypes (sim/skyforge_sim.py).
 */
#ifndef QUAD_SIM_H
#define QUAD_SIM_H

#include "../src/core/fc.h"

typedef struct {
    /* airframe (defaults: generic 5" 6S-class freestyle quad, ~650 g) */
    float mass, ixx, iyy, izz, arm_xy;
    float thrust_max, kq, motor_tau, drag_lin, drag_rot;
    /* sensors */
    float gyro_noise, accel_noise;
    vec3_t gyro_bias;
    float vib_amp, vib_hz;        /* rad/s vibration at vib_hz, scaled by throttle */
    uint32_t rc_period_us;
    /* true state */
    vec3_t pos, vel, w;
    quat_t q;
    float s[4];
    bool on_ground;
    vec3_t dist_torque;
    float t;
    uint32_t now_us;
    uint32_t last_rc_us;
    uint32_t rng;
    bool link_up;
    /* flight controller under test */
    fc_config_t cfg;
    fc_t fc;
    rc_command_t rc;
} quad_sim_t;

#ifdef _WIN32
#define SIM_API __declspec(dllexport)
#else
#define SIM_API __attribute__((visibility("default")))
#endif

SIM_API quad_sim_t *sim_create(void);
SIM_API void  sim_destroy(quad_sim_t *s);
SIM_API void  sim_reset(quad_sim_t *s, uint32_t seed);
SIM_API int   sim_set_param(quad_sim_t *s, const char *name, float value);
SIM_API void  sim_set_rc(quad_sim_t *s, float roll, float pitch, float yaw, float throttle,
                         int arm, int angle_mode);
SIM_API void  sim_set_link(quad_sim_t *s, int up);
SIM_API void  sim_set_disturbance(quad_sim_t *s, float tx, float ty, float tz);
SIM_API void  sim_kick_rates(quad_sim_t *s, float p_dps, float q_dps, float r_dps);
SIM_API void  sim_step(quad_sim_t *s, int n);
SIM_API void  sim_run_seconds(quad_sim_t *s, float seconds);
/* Disarmed on the ground with sticks centred and arm switch off for long
 * enough to finish gyro calibration, then flip the arm switch. */
SIM_API int   sim_arm(quad_sim_t *s);
/*
 * out[0]  t (s)
 * out[1..3]  true roll, pitch, yaw (deg)
 * out[4..6]  true body rates p, q, r (deg/s)
 * out[7..9]  estimated roll, pitch, yaw (deg)
 * out[10..12] rate setpoint (deg/s)
 * out[13..16] motor commands [0,1]
 * out[17] altitude (m, up positive)  out[18] armed  out[19] failsafe state
 */
#define SIM_STATE_LEN 20
SIM_API void  sim_get_state(const quad_sim_t *s, float out[SIM_STATE_LEN]);
SIM_API float sim_hover_throttle(const quad_sim_t *s);

#endif

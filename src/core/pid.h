/*
 * pid.h - inner rate PID (one per axis) and outer angle controller.
 *
 * Rate loop (per axis, deg/s in, normalised mixer torque out):
 *   e   = sp - gyro
 *   P   = kp * P_SCALE * e                        * tpa
 *   I  += ki * I_SCALE * e * dt   (conditional integration + clamp)
 *   D   = -kd * D_SCALE * d(gyro)/dt  (on measurement, PT1 filtered) * tpa
 *   FF  =  kf * F_SCALE * d(sp)/dt    (PT1 smoothed)
 * The scale constants make the integer-ish gains feel like Betaflight's
 * (P~45, I~80, D~35, F~120 on a 5" quad), but they are NOT identical.
 *
 * TPA (throttle PID attenuation) linearly reduces P and D above a
 * throttle breakpoint:  tpa = 1 - rate * (thr - bp) / (1 - bp).
 */
#ifndef PID_H
#define PID_H

#include "fc_math.h"
#include "filters.h"

#define PID_P_SCALE 3.2e-5f   /* per deg/s                  */
#define PID_I_SCALE 2.44e-4f  /* per deg                    */
#define PID_D_SCALE 5.29e-7f  /* per deg/s^2                */
#define PID_F_SCALE 1.4e-7f   /* per deg/s^2 of setpoint    */

typedef struct {
    float kp, ki, kd, kf;
} pid_gains_t;

typedef struct {
    pid_gains_t g;
    float i_limit;    /* |I| clamp, normalised output units */
    float out_limit;  /* |P+I+D+F| clamp */
    float integ;
    float prev_meas;
    float prev_sp;
    pt1_t dterm_lpf;
    pt1_t ff_lpf;
    bool  initialised;
    /* last terms (telemetry / blackbox) */
    float p, i, d, f, out;
} pid_axis_t;

void  pid_axis_init(pid_axis_t *a, pid_gains_t g, float loop_hz,
                    float dterm_lpf_hz, float ff_lpf_hz);
void  pid_axis_reset(pid_axis_t *a);
/*
 * saturated: mixer could not realise the previous command on this loop.
 * When set, the integrator is only allowed to move towards zero.
 */
float pid_axis_update(pid_axis_t *a, float sp_dps, float meas_dps, float dt,
                      float tpa, bool saturated);

float pid_tpa_factor(float throttle, float breakpoint, float rate);

/* Stick (-1..1) to rate (deg/s): "actual rates"-like curve. */
float rc_rate_curve(float stick, float center_dps, float max_dps, float expo);

/*
 * Angle mode outer loop. Desired attitude = (roll_sp, pitch_sp, current
 * yaw). Error quaternion q_e = q^-1 (x) q_des, rate_sp = kp * rotvec(q_e)
 * (body frame). Returns roll/pitch rate setpoints in deg/s, limited.
 */
vec3_t angle_controller(quat_t q, float roll_sp_rad, float pitch_sp_rad,
                        float kp, float max_rate_dps);

#endif

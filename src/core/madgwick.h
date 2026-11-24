/*
 * madgwick.h - Madgwick gradient-descent AHRS (IMU variant, no magnetometer).
 *
 * qdot = 1/2 q (x) [0, w]  -  beta * grad f / |grad f|
 * f(q) = R(q)^T e3 - v,   v = measured "down" direction in body frame
 *                            = -accel / |accel|  (accelerometers measure
 *                              specific force, which points UP at rest)
 */
#ifndef MADGWICK_H
#define MADGWICK_H

#include "fc_math.h"

typedef struct {
    quat_t q;
    float beta;          /* steady-state gain (rad/s), typical 0.02 .. 0.1 */
    float beta_boot;     /* high gain for fast initial convergence */
    float boot_time_s;   /* how long beta_boot is used */
    float t;             /* elapsed time */
    float acc_reject;    /* reject accel if | |a|-1g | > acc_reject (g) */
    bool  last_used_acc; /* diagnostics: was accel fused on last update */
} madgwick_t;

void madgwick_init(madgwick_t *m, float beta);
/* Initialise roll/pitch directly from one accel sample (yaw = 0). */
void madgwick_align(madgwick_t *m, vec3_t accel_g);
/*
 * gyro: body rates in rad/s (FRD); accel: specific force in g (FRD).
 * Falls back to pure gyro integration when accel is invalid
 * (zero / far from 1 g, e.g. during hard manoeuvres or free-fall).
 */
void madgwick_update(madgwick_t *m, vec3_t gyro, vec3_t accel_g, float dt);

#endif

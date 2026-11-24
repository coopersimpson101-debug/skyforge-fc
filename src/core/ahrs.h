/*
 * ahrs.h - selectable attitude estimator front-end (Madgwick or EKF).
 */
#ifndef AHRS_H
#define AHRS_H

#include "ekf_att.h"
#include "madgwick.h"

typedef enum {
    AHRS_MADGWICK = 0,
    AHRS_EKF = 1,
} ahrs_type_e;

typedef struct {
    ahrs_type_e type;
    madgwick_t mw;
    ekf_att_t ekf;
    bool aligned;
    /* Above this body rate (rad/s) the accelerometer is ignored and both
     * filters run gyro-only: during fast manoeuvres it measures thrust and
     * drag, not gravity. 0 disables the gate. */
    float acc_rate_gate;
} ahrs_t;

void   ahrs_init(ahrs_t *a, ahrs_type_e type, float madgwick_beta);
void   ahrs_update(ahrs_t *a, vec3_t gyro_rads, vec3_t accel_g, float dt);
quat_t ahrs_quat(const ahrs_t *a);
/* Gyro bias estimate (only the EKF estimates bias; Madgwick returns 0). */
vec3_t ahrs_bias(const ahrs_t *a);
bool   ahrs_used_accel(const ahrs_t *a);

#endif

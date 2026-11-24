/*
 * ekf_att.h - quaternion error-state (multiplicative) EKF for attitude.
 *
 * Nominal state : q (body->world), b (gyro bias, rad/s)
 * Error state   : dx = [dtheta(3), db(3)],  q_true = q (x) exp(dtheta),
 *                 b_true = b + db
 *
 * Predict (gyro, rate w_m):
 *   w = w_m - b
 *   q <- q (x) exp(w dt)
 *   Phi = [ I - [w]x dt   -I dt ]      P <- Phi P Phi^T + Q
 *         [ 0              I    ]
 * Update (accelerometer, unit down vector z = -a/|a|):
 *   h(q) = R^T e3 = g,   H = [ [g]x   0 ]
 *   S = H P H^T + R,  d2 = y^T S^-1 y  (chi-square gate, 3 dof)
 *   K = P H^T S^-1,  dx = K y,  y = z - g
 *   P <- (I - K H) P (I - K H)^T + K R K^T      (Joseph form)
 *   inject: q <- q (x) exp(dtheta), b <- b + db, dx <- 0
 *
 * Yaw (and yaw-axis bias when level) is unobservable without a
 * magnetometer; its covariance is clamped so it cannot grow unbounded.
 */
#ifndef EKF_ATT_H
#define EKF_ATT_H

#include "fc_math.h"

#define EKF_N 6

typedef struct {
    quat_t q;
    vec3_t bias;
    float P[EKF_N][EKF_N];
    /* tuning */
    float gyro_noise;     /* rad/s/sqrt(Hz), gyro white-noise density */
    float bias_walk;      /* rad/s/sqrt(s), bias random walk */
    float accel_noise;    /* unit-vector noise std (dimensionless) */
    float acc_reject;     /* skip update if | |a| - 1g | > acc_reject */
    float p_max_angle;    /* covariance clamp for attitude (rad^2) */
    float bias_max;       /* |bias| clamp, rad/s */
    float gate_chi2;      /* reject update if y^T S^-1 y > gate (0 = off) */
    uint32_t gate_max_rejects; /* accept anyway after this many rejections */
    /* diagnostics */
    float last_innov;     /* |z - h| of last fused update */
    bool  last_used_acc;
    uint32_t rejects;     /* consecutive gated (rejected) updates */
    uint32_t total_rejects;
} ekf_att_t;

void ekf_att_init(ekf_att_t *e);
void ekf_att_align(ekf_att_t *e, vec3_t accel_g);
void ekf_att_predict(ekf_att_t *e, vec3_t gyro, float dt);
void ekf_att_update_accel(ekf_att_t *e, vec3_t accel_g);
/* convenience: predict + update */
void ekf_att_step(ekf_att_t *e, vec3_t gyro, vec3_t accel_g, float dt);

#endif

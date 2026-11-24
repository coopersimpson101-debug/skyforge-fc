/*
 * calib.h - still-detecting gyro bias calibration.
 * Collects N samples; if the standard deviation on any axis exceeds the
 * threshold (craft is being moved) the window restarts.
 */
#ifndef CALIB_H
#define CALIB_H

#include "fc_math.h"

typedef struct {
    float mean[3], m2[3];  /* Welford running mean / sum of squares */
    uint32_t n, target;
    float max_std;     /* rad/s */
    vec3_t bias;       /* rad/s */
    bool done;
    uint32_t restarts;
} gyro_cal_t;

void gyro_cal_start(gyro_cal_t *c, uint32_t samples, float max_std_rads);
/* Returns true once calibration completed (bias valid). */
bool gyro_cal_add(gyro_cal_t *c, vec3_t gyro);

#endif

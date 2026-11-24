/*
 * mixer.h - quad-X motor mixer with airmode / desaturation.
 *
 * Motor numbering (Betaflight-compatible, seen from above, nose up):
 *
 *        4 (FL, CW)     2 (FR, CCW)
 *               \     /
 *                 [ ]          x forward, y right
 *               /     \
 *        3 (RL, CCW)    1 (RR, CW)
 *
 * "Props in" default: motors 1 & 4 spin CW, 2 & 3 spin CCW (seen from
 * above). A CCW prop's reaction torque yaws the frame CW = positive yaw
 * (FRD), so positive yaw demand raises motors 2 & 3.
 *
 *   motor = throttle + roll*R + pitch*P + yaw*Y
 *   R: +0.5 on left motors (3,4), -0.5 right  (roll right = lift left side)
 *   P: +0.5 on front motors (2,4), -0.5 rear  (pitch up   = lift front)
 *   Y: +0.5 on CCW motors (2,3), -0.5 CW
 * so a normalised demand of 1.0 on one axis spans the full [0,1] range.
 */
#ifndef MIXER_H
#define MIXER_H

#include <stdbool.h>

#define MIXER_MAX_MOTORS 8

typedef struct {
    float roll, pitch, yaw;
} mixer_rule_t;

typedef struct {
    int motor_count;
    mixer_rule_t rules[MIXER_MAX_MOTORS];
    bool airmode;
    bool yaw_reversed;   /* props-out: flips yaw column */
    /* diagnostics from last mix */
    bool saturated;
    float rpy_scale;     /* <1 when roll/pitch/yaw had to be scaled down */
} mixer_t;

void mixer_init_quadx(mixer_t *m, bool airmode, bool yaw_reversed);
/*
 * throttle in [0,1], roll/pitch/yaw normalised torque demands (~[-1,1]).
 * out[] receives motor commands in [0,1]. Returns true when saturated.
 */
bool mixer_mix(mixer_t *m, float throttle, float roll, float pitch, float yaw,
               float out[]);

#endif

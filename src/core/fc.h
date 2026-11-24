/*
 * fc.h - platform independent flight controller core.
 *
 *   gyro ISR/poll (gyro_rate_hz) --> fc_gyro_sample()  : calibrate, notch, LPF
 *                                     | every pid_denom samples
 *                                     v
 *                                  fc_pid_step()       : RC -> setpoints,
 *                                     AHRS, angle loop, rate PID, mixer,
 *                                     arming, failsafe
 *                                     v
 *                                  fc.motor[] / fc_motor_dshot()
 *
 * The same code runs in the firmware (src/app/main.c) and in the host
 * simulator (sim/quad_sim.c) and unit tests.
 */
#ifndef FC_H
#define FC_H

#include "ahrs.h"
#include "calib.h"
#include "config.h"
#include "filters.h"
#include "mixer.h"
#include "pid.h"

#define FC_MOTORS 4

typedef enum { MODE_ACRO = 0, MODE_ANGLE = 1 } flight_mode_e;

typedef enum {
    FS_IDLE = 0,     /* link OK */
    FS_LANDING,      /* armed, link lost: level + fixed throttle */
    FS_DISARMED,     /* failsafe disarmed the craft, needs arm-switch cycle */
} fs_state_e;

/* arming-disable reasons (bitmask) */
enum {
    ARM_BLOCK_GYRO_CAL   = 1u << 0,
    ARM_BLOCK_RX_LOSS    = 1u << 1,
    ARM_BLOCK_FAILSAFE   = 1u << 2,
    ARM_BLOCK_THROTTLE   = 1u << 3,
    ARM_BLOCK_ANGLE      = 1u << 4,
    ARM_BLOCK_ARM_SWITCH = 1u << 5, /* switch must be cycled off first */
    ARM_BLOCK_CLI        = 1u << 6,
    ARM_BLOCK_BENCH      = 1u << 7,
    ARM_BLOCK_IMU        = 1u << 8,
};

typedef struct {
    float roll, pitch, yaw;  /* -1..1, + = right / forward(nose down) / right */
    float throttle;          /* 0..1 */
    bool arm_switch;
    bool angle_mode;
} rc_command_t;

typedef struct {
    fc_config_t *cfg;
    /* derived timing */
    float gyro_hz, pid_hz;
    int pid_denom;
    int gyro_count;
    /* sensors */
    biquad_t gyro_lpf[3];
    biquad_t gyro_notch[3];
    pt1_t accel_lpf[3];
    gyro_cal_t cal;
    bool imu_ok;
    vec3_t gyro_raw;      /* rad/s, bias removed, unfiltered */
    vec3_t gyro_filt;     /* rad/s, filtered */
    vec3_t accel;         /* g, filtered */
    ahrs_t ahrs;
    quat_t q;
    /* control */
    pid_axis_t pid[3];
    mixer_t mixer;
    rc_command_t rc;
    uint32_t rc_last_us;
    bool rc_ever;
    flight_mode_e mode;
    vec3_t rate_sp;       /* deg/s */
    vec3_t pid_out;
    float throttle;       /* after failsafe override */
    bool armed;
    bool airmode_active;
    uint32_t arm_block;
    bool arm_sw_prev;
    bool cli_active;
    fs_state_e fs;
    uint32_t fs_start_us;
    uint32_t armed_since_us;
    float motor[FC_MOTORS];
    /* bench motor test (disarmed, props off!) */
    float bench[FC_MOTORS];
    uint32_t bench_until_us;
    /* stats */
    uint32_t pid_steps;
    uint32_t now_us;
} fc_t;

void fc_init(fc_t *fc, fc_config_t *cfg);
/* Recompute filters, gains, loop rates after config change (disarmed). */
void fc_apply_config(fc_t *fc);
/*
 * Feed one IMU sample (body FRD, gyro rad/s, accel g). Returns true when
 * a PID step is due on this sample (every pid_denom samples).
 */
bool fc_gyro_sample(fc_t *fc, vec3_t gyro, vec3_t accel);
/* New RC frame (already mapped to rc_command_t). */
void fc_rc_update(fc_t *fc, const rc_command_t *rc, uint32_t now_us);
/* Map raw CRSF channel values (AETR + AUX) to rc_command_t. */
void fc_rc_from_crsf(const fc_config_t *cfg, const uint16_t ch[16], rc_command_t *rc);
/* Outer + inner loops, mixer, arming, failsafe. */
void fc_pid_step(fc_t *fc, uint32_t now_us);
/* Motor DShot values for this frame (0 = stop when disarmed). */
void fc_motor_dshot(const fc_t *fc, uint16_t out[FC_MOTORS]);
/* Bench test a motor while disarmed; auto-stops after timeout. */
bool fc_bench_motor(fc_t *fc, int motor, float value, uint32_t now_us);
/* Human readable arming flags, e.g. "THROTTLE ANGLE". */
void fc_arm_block_str(uint32_t flags, char *buf, int cap);

#endif

/*
 * config.h - persistent flight controller configuration and parameter
 * table used by the CLI (get/set/dump) and by flash storage (save/load).
 */
#ifndef CONFIG_H
#define CONFIG_H

#include <stdbool.h>
#include <stdint.h>

enum { AXIS_ROLL = 0, AXIS_PITCH = 1, AXIS_YAW = 2 };

typedef enum { FS_PROC_DROP = 0, FS_PROC_LAND = 1 } fs_procedure_e;

typedef struct {
    /* rate PID */
    float pid_p[3], pid_i[3], pid_d[3], pid_f[3];
    float i_limit;
    float dterm_lpf_hz;
    float ff_lpf_hz;
    float tpa_rate;
    float tpa_breakpoint;
    /* rates (deg/s) */
    float rate_center[3];
    float rate_max[3];
    float rate_expo[3];
    /* angle mode */
    float angle_p;
    float angle_max_deg;
    float angle_max_rate;
    /* gyro filters */
    float gyro_lpf_hz;
    float gyro_notch_hz;
    float gyro_notch_cutoff_hz;
    float accel_lpf_hz;
    /* attitude estimation */
    uint8_t ahrs_type;        /* 0 = Madgwick, 1 = EKF */
    float madgwick_beta;
    float ekf_gyro_noise;
    float ekf_accel_noise;
    float ekf_bias_walk;
    /* mixer / motors */
    uint8_t airmode;
    uint8_t yaw_reversed;
    float motor_idle;         /* fraction of DShot range, e.g. 0.055 */
    uint16_t dshot_rate;      /* 150/300/600 (kbit/s) */
    /* scheduling */
    uint16_t gyro_rate_hz;
    uint16_t pid_loop_hz;
    /* receiver */
    uint8_t arm_channel;      /* 0-based CRSF channel, default 4 (AUX1) */
    uint8_t mode_channel;     /* default 5 (AUX2) */
    float rc_deadband;
    /* failsafe */
    uint16_t fs_timeout_ms;
    uint8_t fs_procedure;
    float fs_throttle;
    float fs_land_time_s;
    /* arming */
    float arm_max_angle_deg;
    float arm_max_throttle;
    float gyro_cal_max_dps;   /* max std-dev allowed during calibration */
    /* board */
    uint8_t imu_align;
} fc_config_t;

typedef enum { PT_FLOAT = 0, PT_U8, PT_U16 } param_type_e;

typedef struct {
    const char *name;
    param_type_e type;
    uint16_t offset;
    float min, max;
} param_t;

void config_defaults(fc_config_t *c);
/* Clamp every parameter into its [min, max] range. */
void config_sanitize(fc_config_t *c);

int            param_count(void);
const param_t *param_at(int i);
const param_t *param_find(const char *name);
float          param_get(const fc_config_t *c, const param_t *p);
bool           param_set(fc_config_t *c, const param_t *p, float v);

/* ---- persistent storage ------------------------------------------------ */
typedef struct {
    int (*erase)(void *ctx);
    int (*write)(void *ctx, uint32_t offset, const void *data, uint32_t len);
    int (*read)(void *ctx, uint32_t offset, void *data, uint32_t len);
    void *ctx;
} config_storage_t;

#define CONFIG_MAGIC   0x53464643u /* "SFFC" */
#define CONFIG_VERSION 1u

uint32_t crc32_update(uint32_t crc, const void *data, uint32_t len);
bool config_save(const fc_config_t *c, const config_storage_t *st);
/* Returns false (and leaves *c untouched) if no valid config is stored. */
bool config_load(fc_config_t *c, const config_storage_t *st);

#endif

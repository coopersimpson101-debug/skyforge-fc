/*
 * config.c - defaults, parameter table, CRC-protected flash blob.
 */
#include "config.h"
#include "fmt.h"

#define OFF(field) ((uint16_t)__builtin_offsetof(fc_config_t, field))

static const param_t params[] = {
    {"p_roll", PT_FLOAT, OFF(pid_p[0]), 0, 250},
    {"i_roll", PT_FLOAT, OFF(pid_i[0]), 0, 250},
    {"d_roll", PT_FLOAT, OFF(pid_d[0]), 0, 250},
    {"f_roll", PT_FLOAT, OFF(pid_f[0]), 0, 1000},
    {"p_pitch", PT_FLOAT, OFF(pid_p[1]), 0, 250},
    {"i_pitch", PT_FLOAT, OFF(pid_i[1]), 0, 250},
    {"d_pitch", PT_FLOAT, OFF(pid_d[1]), 0, 250},
    {"f_pitch", PT_FLOAT, OFF(pid_f[1]), 0, 1000},
    {"p_yaw", PT_FLOAT, OFF(pid_p[2]), 0, 250},
    {"i_yaw", PT_FLOAT, OFF(pid_i[2]), 0, 250},
    {"d_yaw", PT_FLOAT, OFF(pid_d[2]), 0, 250},
    {"f_yaw", PT_FLOAT, OFF(pid_f[2]), 0, 1000},
    {"i_limit", PT_FLOAT, OFF(i_limit), 0, 1},
    {"dterm_lpf_hz", PT_FLOAT, OFF(dterm_lpf_hz), 0, 1000},
    {"ff_lpf_hz", PT_FLOAT, OFF(ff_lpf_hz), 0, 1000},
    {"tpa_rate", PT_FLOAT, OFF(tpa_rate), 0, 1},
    {"tpa_breakpoint", PT_FLOAT, OFF(tpa_breakpoint), 0, 1},
    {"rate_center_roll", PT_FLOAT, OFF(rate_center[0]), 10, 1000},
    {"rate_center_pitch", PT_FLOAT, OFF(rate_center[1]), 10, 1000},
    {"rate_center_yaw", PT_FLOAT, OFF(rate_center[2]), 10, 1000},
    {"rate_max_roll", PT_FLOAT, OFF(rate_max[0]), 10, 1800},
    {"rate_max_pitch", PT_FLOAT, OFF(rate_max[1]), 10, 1800},
    {"rate_max_yaw", PT_FLOAT, OFF(rate_max[2]), 10, 1800},
    {"rate_expo_roll", PT_FLOAT, OFF(rate_expo[0]), 0, 1},
    {"rate_expo_pitch", PT_FLOAT, OFF(rate_expo[1]), 0, 1},
    {"rate_expo_yaw", PT_FLOAT, OFF(rate_expo[2]), 0, 1},
    {"angle_p", PT_FLOAT, OFF(angle_p), 0, 50},
    {"angle_max_deg", PT_FLOAT, OFF(angle_max_deg), 5, 85},
    {"angle_max_rate", PT_FLOAT, OFF(angle_max_rate), 10, 1000},
    {"gyro_lpf_hz", PT_FLOAT, OFF(gyro_lpf_hz), 0, 2000},
    {"gyro_notch_hz", PT_FLOAT, OFF(gyro_notch_hz), 0, 2000},
    {"gyro_notch_cutoff_hz", PT_FLOAT, OFF(gyro_notch_cutoff_hz), 0, 2000},
    {"accel_lpf_hz", PT_FLOAT, OFF(accel_lpf_hz), 0, 500},
    {"ahrs_type", PT_U8, OFF(ahrs_type), 0, 1},
    {"madgwick_beta", PT_FLOAT, OFF(madgwick_beta), 0, 5},
    {"ekf_gyro_noise", PT_FLOAT, OFF(ekf_gyro_noise), 1e-5f, 1},
    {"ekf_accel_noise", PT_FLOAT, OFF(ekf_accel_noise), 1e-3f, 10},
    {"ekf_bias_walk", PT_FLOAT, OFF(ekf_bias_walk), 0, 1},
    {"airmode", PT_U8, OFF(airmode), 0, 1},
    {"yaw_reversed", PT_U8, OFF(yaw_reversed), 0, 1},
    {"motor_idle", PT_FLOAT, OFF(motor_idle), 0, 0.3f},
    {"dshot_rate", PT_U16, OFF(dshot_rate), 150, 600},
    {"gyro_rate_hz", PT_U16, OFF(gyro_rate_hz), 1000, 8000},
    {"pid_loop_hz", PT_U16, OFF(pid_loop_hz), 500, 8000},
    {"arm_channel", PT_U8, OFF(arm_channel), 4, 15},
    {"mode_channel", PT_U8, OFF(mode_channel), 4, 15},
    {"rc_deadband", PT_FLOAT, OFF(rc_deadband), 0, 0.2f},
    {"fs_timeout_ms", PT_U16, OFF(fs_timeout_ms), 50, 5000},
    {"fs_procedure", PT_U8, OFF(fs_procedure), 0, 1},
    {"fs_throttle", PT_FLOAT, OFF(fs_throttle), 0, 1},
    {"fs_land_time_s", PT_FLOAT, OFF(fs_land_time_s), 0, 60},
    {"arm_max_angle_deg", PT_FLOAT, OFF(arm_max_angle_deg), 0, 180},
    {"arm_max_throttle", PT_FLOAT, OFF(arm_max_throttle), 0, 0.3f},
    {"gyro_cal_max_dps", PT_FLOAT, OFF(gyro_cal_max_dps), 0.1f, 50},
    {"imu_align", PT_U8, OFF(imu_align), 0, 7},
};

#define NPARAMS ((int)(sizeof(params) / sizeof(params[0])))

void config_defaults(fc_config_t *c)
{
    uint8_t *b = (uint8_t *)c;
    for (unsigned i = 0; i < sizeof(*c); i++) {
        b[i] = 0; /* deterministic padding -> stable CRC */
    }
    const float p[3] = {55, 58, 50}, i[3] = {50, 52, 60};
    const float d[3] = {45, 48, 0}, f[3] = {100, 105, 90};
    for (int a = 0; a < 3; a++) {
        c->pid_p[a] = p[a];
        c->pid_i[a] = i[a];
        c->pid_d[a] = d[a];
        c->pid_f[a] = f[a];
        c->rate_center[a] = 200.0f;
        c->rate_max[a] = 670.0f;
        c->rate_expo[a] = 0.54f;
    }
    c->i_limit = 0.25f;
    c->dterm_lpf_hz = 100.0f;
    c->ff_lpf_hz = 40.0f;
    c->tpa_rate = 0.65f;
    c->tpa_breakpoint = 0.65f;
    c->angle_p = 6.0f;
    c->angle_max_deg = 55.0f;
    c->angle_max_rate = 500.0f;
    c->gyro_lpf_hz = 200.0f;
    c->gyro_notch_hz = 0.0f;           /* disabled until tuned on hardware */
    c->gyro_notch_cutoff_hz = 0.0f;
    c->accel_lpf_hz = 20.0f;
    c->ahrs_type = 0;
    c->madgwick_beta = 0.02f;
    c->ekf_gyro_noise = 0.002f;
    c->ekf_accel_noise = 0.5f;
    c->ekf_bias_walk = 1e-4f;
    c->airmode = 1;
    c->yaw_reversed = 0;
    c->motor_idle = 0.055f;
    c->dshot_rate = 600;
    c->gyro_rate_hz = 8000;
    c->pid_loop_hz = 4000;
    c->arm_channel = 4;
    c->mode_channel = 5;
    c->rc_deadband = 0.01f;
    c->fs_timeout_ms = 300;
    c->fs_procedure = FS_PROC_LAND;
    c->fs_throttle = 0.30f;
    c->fs_land_time_s = 5.0f;
    c->arm_max_angle_deg = 25.0f;
    c->arm_max_throttle = 0.05f;
    c->gyro_cal_max_dps = 2.0f;
    c->imu_align = 0;
}

int param_count(void)
{
    return NPARAMS;
}

const param_t *param_at(int i)
{
    return (i >= 0 && i < NPARAMS) ? &params[i] : 0;
}

const param_t *param_find(const char *name)
{
    for (int i = 0; i < NPARAMS; i++) {
        if (fmt_streq(params[i].name, name)) {
            return &params[i];
        }
    }
    return 0;
}

float param_get(const fc_config_t *c, const param_t *p)
{
    const uint8_t *base = (const uint8_t *)c + p->offset;
    switch (p->type) {
    case PT_U8:  return (float)*base;
    case PT_U16: return (float)*(const uint16_t *)(const void *)base;
    default:     return *(const float *)(const void *)base;
    }
}

bool param_set(fc_config_t *c, const param_t *p, float v)
{
    if (v != v || v < p->min || v > p->max) {
        return false;
    }
    uint8_t *base = (uint8_t *)c + p->offset;
    switch (p->type) {
    case PT_U8:
        *base = (uint8_t)(v + 0.5f);
        break;
    case PT_U16:
        *(uint16_t *)(void *)base = (uint16_t)(v + 0.5f);
        break;
    default:
        *(float *)(void *)base = v;
        break;
    }
    return true;
}

void config_sanitize(fc_config_t *c)
{
    for (int i = 0; i < NPARAMS; i++) {
        const param_t *p = &params[i];
        float v = param_get(c, p);
        if (v != v || v < p->min) {
            param_set(c, p, p->min);
        } else if (v > p->max) {
            param_set(c, p, p->max);
        }
    }
}

/* ---- CRC32 (IEEE 802.3, reflected, poly 0xEDB88320) ------------------- */
uint32_t crc32_update(uint32_t crc, const void *data, uint32_t len)
{
    const uint8_t *p = (const uint8_t *)data;
    crc = ~crc;
    for (uint32_t i = 0; i < len; i++) {
        crc ^= p[i];
        for (int b = 0; b < 8; b++) {
            crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
        }
    }
    return ~crc;
}

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t size;
} config_header_t;

bool config_save(const fc_config_t *c, const config_storage_t *st)
{
    config_header_t h = {CONFIG_MAGIC, CONFIG_VERSION, (uint16_t)sizeof(fc_config_t)};
    uint32_t crc = crc32_update(0, &h, sizeof(h));
    crc = crc32_update(crc, c, sizeof(*c));
    if (st->erase(st->ctx) != 0) {
        return false;
    }
    uint32_t off = 0;
    if (st->write(st->ctx, off, &h, sizeof(h)) != 0) return false;
    off += sizeof(h);
    if (st->write(st->ctx, off, c, sizeof(*c)) != 0) return false;
    off += sizeof(*c);
    if (st->write(st->ctx, off, &crc, sizeof(crc)) != 0) return false;
    /* read back and verify */
    fc_config_t verify;
    return config_load(&verify, st);
}

bool config_load(fc_config_t *c, const config_storage_t *st)
{
    config_header_t h;
    fc_config_t tmp;
    uint32_t crc_stored;
    if (st->read(st->ctx, 0, &h, sizeof(h)) != 0) return false;
    if (h.magic != CONFIG_MAGIC || h.version != CONFIG_VERSION ||
        h.size != sizeof(fc_config_t)) {
        return false;
    }
    if (st->read(st->ctx, sizeof(h), &tmp, sizeof(tmp)) != 0) return false;
    if (st->read(st->ctx, sizeof(h) + sizeof(tmp), &crc_stored, sizeof(crc_stored)) != 0) {
        return false;
    }
    uint32_t crc = crc32_update(0, &h, sizeof(h));
    crc = crc32_update(crc, &tmp, sizeof(tmp));
    if (crc != crc_stored) {
        return false;
    }
    *c = tmp;
    config_sanitize(c);
    return true;
}

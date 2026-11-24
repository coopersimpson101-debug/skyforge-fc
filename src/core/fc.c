/*
 * fc.c - flight controller core: sensor pipeline, cascaded control loops,
 * arming state machine and failsafe. Platform independent.
 */
#include "fc.h"
#include "crsf.h"
#include "dshot.h"
#include "fmt.h"

#define BENCH_TIMEOUT_US 2000000u
#define AIRMODE_START_THROTTLE 0.25f

void fc_apply_config(fc_t *fc)
{
    fc_config_t *c = fc->cfg;
    fc->gyro_hz = (float)c->gyro_rate_hz;
    int denom = (int)(fc->gyro_hz / (float)c->pid_loop_hz + 0.5f);
    if (denom < 1) denom = 1;
    if (denom > 8) denom = 8;
    fc->pid_denom = denom;
    fc->pid_hz = fc->gyro_hz / (float)denom;
    fc->gyro_count = 0;

    const float nq = biquad_notch_q(c->gyro_notch_hz, c->gyro_notch_cutoff_hz);
    for (int i = 0; i < 3; i++) {
        biquad_lpf_init(&fc->gyro_lpf[i], c->gyro_lpf_hz, fc->gyro_hz, BIQUAD_Q_BUTTERWORTH);
        biquad_notch_init(&fc->gyro_notch[i], c->gyro_notch_hz, fc->gyro_hz, nq);
        pt1_init(&fc->accel_lpf[i], c->accel_lpf_hz, fc->gyro_hz);
    }
    fc->accel_lpf[2].y = -1.0f; /* start at level */

    ahrs_init(&fc->ahrs, c->ahrs_type ? AHRS_EKF : AHRS_MADGWICK, c->madgwick_beta);
    fc->ahrs.ekf.gyro_noise = c->ekf_gyro_noise;
    fc->ahrs.ekf.accel_noise = c->ekf_accel_noise;
    fc->ahrs.ekf.bias_walk = c->ekf_bias_walk;

    for (int a = 0; a < 3; a++) {
        pid_gains_t g = {c->pid_p[a], c->pid_i[a], c->pid_d[a], c->pid_f[a]};
        pid_axis_init(&fc->pid[a], g, fc->pid_hz, c->dterm_lpf_hz, c->ff_lpf_hz);
        fc->pid[a].i_limit = c->i_limit;
    }
    mixer_init_quadx(&fc->mixer, c->airmode != 0, c->yaw_reversed != 0);
}

void fc_init(fc_t *fc, fc_config_t *cfg)
{
    uint8_t *b = (uint8_t *)fc;
    for (unsigned i = 0; i < sizeof(*fc); i++) {
        b[i] = 0;
    }
    fc->cfg = cfg;
    fc_apply_config(fc);
    /* 1 s of still samples for the gyro bias */
    gyro_cal_start(&fc->cal, (uint32_t)fc->gyro_hz, cfg->gyro_cal_max_dps * FM_DEG2RAD);
    fc->imu_ok = true;
    fc->q = q_identity();
    fc->accel = v3(0.0f, 0.0f, -1.0f);
    fc->arm_sw_prev = true; /* switch must be seen OFF before first arm */
    fc->mode = MODE_ACRO;
    fc->fs = FS_IDLE;
}

bool fc_gyro_sample(fc_t *fc, vec3_t gyro, vec3_t accel)
{
    if (!fc->cal.done) {
        gyro_cal_add(&fc->cal, gyro);
    }
    vec3_t g = v3_sub(gyro, fc->cal.bias);
    fc->gyro_raw = g;
    float in[3] = {g.x, g.y, g.z}, out[3];
    for (int i = 0; i < 3; i++) {
        float v = biquad_apply(&fc->gyro_notch[i], in[i]);
        out[i] = biquad_apply(&fc->gyro_lpf[i], v);
    }
    fc->gyro_filt = v3(out[0], out[1], out[2]);
    fc->accel = v3(pt1_apply(&fc->accel_lpf[0], accel.x),
                   pt1_apply(&fc->accel_lpf[1], accel.y),
                   pt1_apply(&fc->accel_lpf[2], accel.z));
    if (++fc->gyro_count >= fc->pid_denom) {
        fc->gyro_count = 0;
        return true;
    }
    return false;
}

void fc_rc_update(fc_t *fc, const rc_command_t *rc, uint32_t now_us)
{
    fc->rc = *rc;
    fc->rc_last_us = now_us;
    fc->rc_ever = true;
}

void fc_rc_from_crsf(const fc_config_t *cfg, const uint16_t ch[16], rc_command_t *rc)
{
    /* AETR channel order: 0 aileron, 1 elevator, 2 throttle, 3 rudder */
    rc->roll = crsf_ch_to_norm(ch[0]);
    rc->pitch = crsf_ch_to_norm(ch[1]);
    rc->throttle = 0.5f * (crsf_ch_to_norm(ch[2]) + 1.0f);
    rc->yaw = crsf_ch_to_norm(ch[3]);
    rc->arm_switch = crsf_ch_to_us(ch[cfg->arm_channel & 15]) > 1700.0f;
    rc->angle_mode = crsf_ch_to_us(ch[cfg->mode_channel & 15]) > 1300.0f;
}

static float deadband(float v, float db)
{
    if (fm_absf(v) <= db) {
        return 0.0f;
    }
    /* rescale so output still reaches +-1 */
    return (v - fm_signf(v) * db) / (1.0f - db);
}

static void disarm(fc_t *fc)
{
    fc->armed = false;
    for (int a = 0; a < 3; a++) {
        pid_axis_reset(&fc->pid[a]);
    }
}

static bool bench_active(const fc_t *fc)
{
    return fc->bench_until_us != 0 &&
           (int32_t)(fc->bench_until_us - fc->now_us) > 0;
}

void fc_pid_step(fc_t *fc, uint32_t now_us)
{
    fc_config_t *c = fc->cfg;
    const float dt = 1.0f / fc->pid_hz;
    fc->now_us = now_us;
    fc->pid_steps++;

    /* ---- attitude estimate ---------------------------------------- */
    ahrs_update(&fc->ahrs, fc->gyro_filt, fc->accel, dt);
    fc->q = ahrs_quat(&fc->ahrs);

    /* ---- link health + failsafe ------------------------------------ */
    const bool rx_ok = fc->rc_ever &&
                       (uint32_t)(now_us - fc->rc_last_us) <= (uint32_t)c->fs_timeout_ms * 1000u;
    if (fc->armed && !rx_ok && fc->fs == FS_IDLE) {
        if (c->fs_procedure == FS_PROC_DROP) {
            disarm(fc);
            fc->fs = FS_DISARMED;
        } else {
            fc->fs = FS_LANDING;
            fc->fs_start_us = now_us;
        }
    }
    if (fc->fs == FS_LANDING) {
        if (!fc->armed ||
            (float)(uint32_t)(now_us - fc->fs_start_us) >= c->fs_land_time_s * 1e6f) {
            disarm(fc);
            fc->fs = FS_DISARMED;
        }
    }
    if (fc->fs == FS_DISARMED && rx_ok && !fc->rc.arm_switch) {
        fc->fs = FS_IDLE; /* link back and pilot cycled the arm switch */
    }

    /* ---- arming checks --------------------------------------------- */
    const float tilt_deg = q_tilt_angle(fc->q) * FM_RAD2DEG;
    uint32_t blk = 0;
    if (!fc->cal.done) blk |= ARM_BLOCK_GYRO_CAL;
    if (!fc->imu_ok) blk |= ARM_BLOCK_IMU;
    if (!rx_ok) blk |= ARM_BLOCK_RX_LOSS;
    if (fc->fs != FS_IDLE) blk |= ARM_BLOCK_FAILSAFE;
    if (fc->rc.throttle > c->arm_max_throttle) blk |= ARM_BLOCK_THROTTLE;
    if (tilt_deg > c->arm_max_angle_deg) blk |= ARM_BLOCK_ANGLE;
    if (fc->cli_active) blk |= ARM_BLOCK_CLI;
    if (bench_active(fc)) blk |= ARM_BLOCK_BENCH;

    if (rx_ok) {
        const bool sw = fc->rc.arm_switch;
        if (!fc->armed) {
            if (sw && !fc->arm_sw_prev && blk == 0) {
                fc->armed = true;
                fc->armed_since_us = now_us;
                fc->airmode_active = false;
                for (int a = 0; a < 3; a++) {
                    pid_axis_reset(&fc->pid[a]);
                }
            } else if (sw) {
                blk |= ARM_BLOCK_ARM_SWITCH;
            }
        } else if (!sw) {
            /* pilot disarm always wins, also during a failsafe landing
             * once the link is back */
            disarm(fc);
            if (fc->fs == FS_LANDING) {
                fc->fs = FS_IDLE;
            }
        }
        fc->arm_sw_prev = sw;
    }
    fc->arm_block = blk;

    /* ---- setpoints ------------------------------------------------- */
    rc_command_t rc = fc->rc;
    if (fc->fs == FS_LANDING) {
        rc.roll = rc.pitch = rc.yaw = 0.0f;
        rc.throttle = c->fs_throttle;
        rc.angle_mode = true;
    }
    fc->mode = rc.angle_mode ? MODE_ANGLE : MODE_ACRO;
    const float sr = deadband(rc.roll, c->rc_deadband);
    const float sp = deadband(rc.pitch, c->rc_deadband);
    const float sy = deadband(rc.yaw, c->rc_deadband);
    fc->throttle = fm_clampf(rc.throttle, 0.0f, 1.0f);

    vec3_t rate_sp;
    rate_sp.z = rc_rate_curve(sy, c->rate_center[2], c->rate_max[2], c->rate_expo[2]);
    if (fc->mode == MODE_ANGLE) {
        const float amax = c->angle_max_deg * FM_DEG2RAD;
        /* stick forward (+pitch) = nose down = negative pitch angle */
        vec3_t r = angle_controller(fc->q, sr * amax, -sp * amax, c->angle_p, c->angle_max_rate);
        rate_sp.x = r.x;
        rate_sp.y = r.y;
    } else {
        rate_sp.x = rc_rate_curve(sr, c->rate_center[0], c->rate_max[0], c->rate_expo[0]);
        rate_sp.y = -rc_rate_curve(sp, c->rate_center[1], c->rate_max[1], c->rate_expo[1]);
    }
    fc->rate_sp = rate_sp;

    /* ---- inner rate loop + mixer ----------------------------------- */
    if (fc->armed) {
        if (fc->throttle > AIRMODE_START_THROTTLE) {
            fc->airmode_active = true;
        }
        const bool hold_i = c->airmode ? !fc->airmode_active : (fc->throttle < 0.02f);
        const float tpa = pid_tpa_factor(fc->throttle, c->tpa_breakpoint, c->tpa_rate);
        const float sp3[3] = {rate_sp.x, rate_sp.y, rate_sp.z};
        const float gy[3] = {fc->gyro_filt.x * FM_RAD2DEG, fc->gyro_filt.y * FM_RAD2DEG,
                             fc->gyro_filt.z * FM_RAD2DEG};
        float out[3];
        for (int a = 0; a < 3; a++) {
            out[a] = pid_axis_update(&fc->pid[a], sp3[a], gy[a], dt,
                                     a == AXIS_YAW ? 1.0f : tpa, fc->mixer.saturated);
            if (hold_i) {
                fc->pid[a].integ = 0.0f;
            }
        }
        fc->pid_out = v3(out[0], out[1], out[2]);
        mixer_mix(&fc->mixer, fc->throttle, out[0], out[1], out[2], fc->motor);
    } else {
        fc->pid_out = v3(0.0f, 0.0f, 0.0f);
        fc->mixer.saturated = false;
        const bool bench = bench_active(fc);
        for (int i = 0; i < FC_MOTORS; i++) {
            fc->motor[i] = bench ? fc->bench[i] : 0.0f;
        }
        if (!bench) {
            fc->bench_until_us = 0;
        }
    }
}

void fc_motor_dshot(const fc_t *fc, uint16_t out[FC_MOTORS])
{
    const bool bench = !fc->armed && bench_active(fc);
    for (int i = 0; i < FC_MOTORS; i++) {
        if (fc->armed) {
            out[i] = dshot_value_from_motor(fc->motor[i], fc->cfg->motor_idle);
        } else if (bench && fc->bench[i] > 0.0f) {
            out[i] = dshot_value_from_motor(fc->bench[i], fc->cfg->motor_idle);
        } else {
            out[i] = DSHOT_CMD_MOTOR_STOP;
        }
    }
}

bool fc_bench_motor(fc_t *fc, int motor, float value, uint32_t now_us)
{
    if (fc->armed || motor < -1 || motor >= FC_MOTORS) {
        return false;
    }
    value = fm_clampf(value, 0.0f, 0.5f); /* bench limit */
    for (int i = 0; i < FC_MOTORS; i++) {
        if (motor == -1 || motor == i) {
            fc->bench[i] = value;
        }
    }
    fc->now_us = now_us;
    fc->bench_until_us = (now_us + BENCH_TIMEOUT_US) | 1u;
    return true;
}

void fc_arm_block_str(uint32_t flags, char *buf, int cap)
{
    static const char *names[] = {"GYRO_CAL", "RX_LOSS", "FAILSAFE", "THROTTLE", "ANGLE",
                                  "ARM_SWITCH", "CLI", "BENCH", "IMU"};
    buf[0] = 0;
    if (flags == 0) {
        fmt_append(buf, cap, "NONE");
        return;
    }
    for (unsigned i = 0; i < sizeof(names) / sizeof(names[0]); i++) {
        if (flags & (1u << i)) {
            if (buf[0]) fmt_append(buf, cap, " ");
            fmt_append(buf, cap, names[i]);
        }
    }
}

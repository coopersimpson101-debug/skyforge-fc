/*
 * fc_math.c - freestanding float math (no libm), vectors and quaternions.
 * All polynomial approximations are tested against libm on the host
 * (tests/test_math.c).
 */
#include "fc_math.h"

float fm_sqrtf(float x)
{
    if (x <= 0.0f) {
        return 0.0f;
    }
#if defined(__ARM_FP) && !defined(FC_HOST)
    float r;
    __asm__("vsqrt.f32 %0, %1" : "=t"(r) : "t"(x));
    return r;
#elif defined(__GNUC__) || defined(__clang__)
    return __builtin_sqrtf(x);
#else
    /* Newton-Raphson fallback */
    float r = x > 1.0f ? x * 0.5f : 1.0f;
    for (int i = 0; i < 20; i++) {
        r = 0.5f * (r + x / r);
    }
    return r;
#endif
}

/* sin on [-pi/2, pi/2] by odd Taylor polynomial up to x^11 (err < 6e-8). */
static float sin_poly(float x)
{
    const float x2 = x * x;
    /* Horner form of x - x^3/3! + x^5/5! - x^7/7! + x^9/9! - x^11/11! */
    float p = -2.5052108e-8f;          /* -1/11! */
    p = p * x2 + 2.7557319e-6f;        /*  1/9!  */
    p = p * x2 - 1.9841270e-4f;        /* -1/7!  */
    p = p * x2 + 8.3333333e-3f;        /*  1/5!  */
    p = p * x2 - 1.6666667e-1f;        /* -1/3!  */
    return x + x * x2 * p;
}

static float wrap_pi(float x)
{
    /* reduce to [-pi, pi] */
    if (x > FM_PI || x < -FM_PI) {
        float k = x / FM_TWO_PI;
        /* round to nearest integer without libm */
        int32_t n = (int32_t)(k + (k >= 0.0f ? 0.5f : -0.5f));
        x -= (float)n * FM_TWO_PI;
    }
    return x;
}

float fm_sinf(float x)
{
    x = wrap_pi(x);
    if (x > FM_HALF_PI) {
        x = FM_PI - x;
    } else if (x < -FM_HALF_PI) {
        x = -FM_PI - x;
    }
    return sin_poly(x);
}

float fm_cosf(float x)
{
    return fm_sinf(x + FM_HALF_PI);
}

/* atan on |t| <= tan(pi/8) by odd series up to t^15 (err < 1e-8). */
static float atan_poly(float t)
{
    const float t2 = t * t;
    float p = -1.0f / 15.0f;
    p = p * t2 + 1.0f / 13.0f;
    p = p * t2 - 1.0f / 11.0f;
    p = p * t2 + 1.0f / 9.0f;
    p = p * t2 - 1.0f / 7.0f;
    p = p * t2 + 1.0f / 5.0f;
    p = p * t2 - 1.0f / 3.0f;
    return t + t * t2 * p;
}

float fm_atanf(float x)
{
    const float TAN_PI_8 = 0.41421356f;
    float sign = 1.0f;
    if (x < 0.0f) {
        sign = -1.0f;
        x = -x;
    }
    bool inv = false;
    if (x > 1.0f) {
        inv = true;
        x = 1.0f / x;
    }
    float r;
    if (x > TAN_PI_8) {
        /* atan(x) = pi/4 + atan((x-1)/(x+1)) */
        r = 0.25f * FM_PI + atan_poly((x - 1.0f) / (x + 1.0f));
    } else {
        r = atan_poly(x);
    }
    if (inv) {
        r = FM_HALF_PI - r;
    }
    return sign * r;
}

float fm_atan2f(float y, float x)
{
    if (x > 0.0f) {
        return fm_atanf(y / x);
    }
    if (x < 0.0f) {
        return fm_atanf(y / x) + (y >= 0.0f ? FM_PI : -FM_PI);
    }
    if (y > 0.0f) {
        return FM_HALF_PI;
    }
    if (y < 0.0f) {
        return -FM_HALF_PI;
    }
    return 0.0f;
}

float fm_asinf(float x)
{
    x = fm_clampf(x, -1.0f, 1.0f);
    return fm_atan2f(x, fm_sqrtf(1.0f - x * x));
}

float fm_acosf(float x)
{
    x = fm_clampf(x, -1.0f, 1.0f);
    return fm_atan2f(fm_sqrtf(1.0f - x * x), x);
}

vec3_t v3_normalize(vec3_t a)
{
    float n = v3_norm(a);
    if (n < 1e-12f) {
        return v3(0.0f, 0.0f, 0.0f);
    }
    return v3_scale(a, 1.0f / n);
}

quat_t q_mul(quat_t a, quat_t b)
{
    quat_t r;
    r.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
    r.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
    r.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x;
    r.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w;
    return r;
}

quat_t q_normalize(quat_t q)
{
    float n = fm_sqrtf(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    if (n < 1e-12f) {
        return q_identity();
    }
    float s = 1.0f / n;
    quat_t r = {q.w * s, q.x * s, q.y * s, q.z * s};
    return r;
}

vec3_t q_rotate(quat_t q, vec3_t v)
{
    /* v' = v + 2w(u x v) + 2 u x (u x v), u = vector part */
    vec3_t u = v3(q.x, q.y, q.z);
    vec3_t t = v3_scale(v3_cross(u, v), 2.0f);
    return v3_add(v3_add(v, v3_scale(t, q.w)), v3_cross(u, t));
}

vec3_t q_rotate_inv(quat_t q, vec3_t v)
{
    return q_rotate(q_conj(q), v);
}

quat_t q_from_rotvec(vec3_t rv)
{
    float angle = v3_norm(rv);
    quat_t q;
    if (angle < 1e-6f) {
        /* second-order small-angle expansion */
        q.w = 1.0f - angle * angle * 0.125f;
        q.x = 0.5f * rv.x;
        q.y = 0.5f * rv.y;
        q.z = 0.5f * rv.z;
        return q_normalize(q);
    }
    float half = 0.5f * angle;
    float s = fm_sinf(half) / angle;
    q.w = fm_cosf(half);
    q.x = rv.x * s;
    q.y = rv.y * s;
    q.z = rv.z * s;
    return q;
}

vec3_t q_to_rotvec(quat_t q)
{
    if (q.w < 0.0f) {
        q.w = -q.w; q.x = -q.x; q.y = -q.y; q.z = -q.z;
    }
    float s = fm_sqrtf(q.x * q.x + q.y * q.y + q.z * q.z);
    if (s < 1e-7f) {
        return v3(2.0f * q.x, 2.0f * q.y, 2.0f * q.z);
    }
    float angle = 2.0f * fm_atan2f(s, q.w);
    return v3_scale(v3(q.x, q.y, q.z), angle / s);
}

quat_t q_from_euler(float roll, float pitch, float yaw)
{
    float cr = fm_cosf(0.5f * roll), sr = fm_sinf(0.5f * roll);
    float cp = fm_cosf(0.5f * pitch), sp = fm_sinf(0.5f * pitch);
    float cy = fm_cosf(0.5f * yaw), sy = fm_sinf(0.5f * yaw);
    quat_t q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;
    return q;
}

void q_to_euler(quat_t q, float *roll, float *pitch, float *yaw)
{
    if (roll) {
        *roll = fm_atan2f(2.0f * (q.w * q.x + q.y * q.z),
                          1.0f - 2.0f * (q.x * q.x + q.y * q.y));
    }
    if (pitch) {
        *pitch = fm_asinf(2.0f * (q.w * q.y - q.z * q.x));
    }
    if (yaw) {
        *yaw = fm_atan2f(2.0f * (q.w * q.z + q.x * q.y),
                         1.0f - 2.0f * (q.y * q.y + q.z * q.z));
    }
}

vec3_t q_body_down(quat_t q)
{
    /* third row of the body->world rotation matrix */
    return v3(2.0f * (q.x * q.z - q.w * q.y),
              2.0f * (q.y * q.z + q.w * q.x),
              1.0f - 2.0f * (q.x * q.x + q.y * q.y));
}

float q_tilt_angle(quat_t q)
{
    return fm_acosf(q_body_down(q).z);
}

float q_angle_between(quat_t a, quat_t b)
{
    quat_t d = q_mul(q_conj(a), b);
    float w = fm_clampf(fm_absf(d.w), 0.0f, 1.0f);
    return 2.0f * fm_acosf(w);
}

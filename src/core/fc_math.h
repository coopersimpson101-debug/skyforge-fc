/*
 * fc_math.h - freestanding float math, 3-vectors and quaternions.
 *
 * The firmware is built with -ffreestanding and no libc/libm, so every
 * transcendental function used by the flight code is implemented here.
 *
 * Frame conventions used throughout skyforge-fc:
 *   world frame  : NED  (x north, y east, z down)
 *   body frame   : FRD  (x forward, y right, z down)
 *   quaternion q : rotates body -> world,  v_w = q (x) v_b (x) q*
 *                  stored as {w, x, y, z}
 *   positive roll  = right wing down, positive pitch = nose up,
 *   positive yaw   = nose right (clockwise seen from above).
 */
#ifndef FC_MATH_H
#define FC_MATH_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define FM_PI       3.14159265358979323846f
#define FM_TWO_PI   6.28318530717958647692f
#define FM_HALF_PI  1.57079632679489661923f
#define FM_DEG2RAD  (FM_PI / 180.0f)
#define FM_RAD2DEG  (180.0f / FM_PI)
#define FM_GRAVITY  9.80665f

typedef struct { float x, y, z; } vec3_t;
typedef struct { float w, x, y, z; } quat_t;

/* ---- scalar ---------------------------------------------------------- */
float fm_sqrtf(float x);
float fm_sinf(float x);
float fm_cosf(float x);
float fm_atanf(float x);
float fm_atan2f(float y, float x);
float fm_asinf(float x);
float fm_acosf(float x);

static inline float fm_absf(float x) { return x < 0.0f ? -x : x; }
static inline float fm_minf(float a, float b) { return a < b ? a : b; }
static inline float fm_maxf(float a, float b) { return a > b ? a : b; }
static inline float fm_clampf(float x, float lo, float hi)
{
    return x < lo ? lo : (x > hi ? hi : x);
}
static inline float fm_signf(float x) { return x < 0.0f ? -1.0f : 1.0f; }
static inline float fm_sq(float x) { return x * x; }
/* Fast inverse sqrt using the hardware sqrt (exact enough for filters). */
static inline float fm_invsqrtf(float x) { return 1.0f / fm_sqrtf(x); }

/* ---- vec3 ------------------------------------------------------------ */
static inline vec3_t v3(float x, float y, float z) { vec3_t v = {x, y, z}; return v; }
static inline vec3_t v3_add(vec3_t a, vec3_t b) { return v3(a.x + b.x, a.y + b.y, a.z + b.z); }
static inline vec3_t v3_sub(vec3_t a, vec3_t b) { return v3(a.x - b.x, a.y - b.y, a.z - b.z); }
static inline vec3_t v3_scale(vec3_t a, float s) { return v3(a.x * s, a.y * s, a.z * s); }
static inline float  v3_dot(vec3_t a, vec3_t b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
static inline vec3_t v3_cross(vec3_t a, vec3_t b)
{
    return v3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}
static inline float v3_norm(vec3_t a) { return fm_sqrtf(v3_dot(a, a)); }
vec3_t v3_normalize(vec3_t a);

/* ---- quaternion ------------------------------------------------------ */
static inline quat_t q_identity(void) { quat_t q = {1.0f, 0.0f, 0.0f, 0.0f}; return q; }
static inline quat_t q_conj(quat_t q) { quat_t r = {q.w, -q.x, -q.y, -q.z}; return r; }
quat_t q_mul(quat_t a, quat_t b);
quat_t q_normalize(quat_t q);
/* Rotate body vector into world: q v q* */
vec3_t q_rotate(quat_t q, vec3_t v);
/* Rotate world vector into body: q* v q */
vec3_t q_rotate_inv(quat_t q, vec3_t v);
/* Quaternion from rotation vector (axis * angle), exact exponential map. */
quat_t q_from_rotvec(vec3_t rv);
/* Rotation vector (axis*angle) of q, shortest path (w >= 0). */
vec3_t q_to_rotvec(quat_t q);
/* ZYX (yaw-pitch-roll) Euler angles in radians. */
quat_t q_from_euler(float roll, float pitch, float yaw);
void   q_to_euler(quat_t q, float *roll, float *pitch, float *yaw);
/* "Down" direction (world +z) expressed in the body frame: R^T e3. */
vec3_t q_body_down(quat_t q);
/* Tilt angle between body z and world z, radians [0, pi]. */
float  q_tilt_angle(quat_t q);
/* Angle of the rotation a^-1 b, radians [0, pi]. */
float  q_angle_between(quat_t a, quat_t b);

#endif /* FC_MATH_H */

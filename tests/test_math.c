#include "../src/core/fc_math.h"
#include "tinytest.h"

static void test_trig_vs_libm(void)
{
    double max_s = 0, max_c = 0, max_t = 0;
    for (int i = -2000; i <= 2000; i++) {
        float x = (float)i * 0.01f; /* -20 .. 20 rad */
        max_s = fmax(max_s, fabs(fm_sinf(x) - sin(x)));
        max_c = fmax(max_c, fabs(fm_cosf(x) - cos(x)));
    }
    for (int i = -100; i <= 100; i++) {
        for (int j = -100; j <= 100; j++) {
            float y = i * 0.37f, x = j * 0.29f;
            if (i == 0 && j == 0) continue;
            max_t = fmax(max_t, fabs(fm_atan2f(y, x) - atan2(y, x)));
        }
    }
    CHECK(max_s < 2e-6);
    CHECK(max_c < 2e-6);
    CHECK(max_t < 2e-6);
    for (int i = -100; i <= 100; i++) {
        float x = i * 0.01f;
        CHECK_NEAR(fm_asinf(x), asin(x), 3e-6);
        CHECK_NEAR(fm_acosf(x), acos(x), 3e-6);
    }
    CHECK_NEAR(fm_sqrtf(2.0f), 1.41421356, 1e-6);
    CHECK_NEAR(fm_sqrtf(0.0f), 0.0, 0);
}

static void test_quat_euler_roundtrip(void)
{
    for (int r = -170; r <= 170; r += 34) {
        for (int p = -80; p <= 80; p += 20) {
            for (int y = -170; y <= 170; y += 50) {
                quat_t q = q_from_euler(r * FM_DEG2RAD, p * FM_DEG2RAD, y * FM_DEG2RAD);
                float rr, pp, yy;
                q_to_euler(q, &rr, &pp, &yy);
                CHECK_NEAR(rr * FM_RAD2DEG, r, 1e-3);
                CHECK_NEAR(pp * FM_RAD2DEG, p, 1e-3);
                CHECK_NEAR(yy * FM_RAD2DEG, y, 1e-3);
            }
        }
    }
}

static void test_quat_rotation(void)
{
    /* +90 deg yaw: body x (forward) points east in NED */
    quat_t q = q_from_euler(0, 0, FM_HALF_PI);
    vec3_t v = q_rotate(q, v3(1, 0, 0));
    CHECK_NEAR(v.x, 0, 1e-6);
    CHECK_NEAR(v.y, 1, 1e-6);
    vec3_t b = q_rotate_inv(q, v);
    CHECK_NEAR(b.x, 1, 1e-6);
    CHECK_NEAR(b.y, 0, 1e-6);
    /* body_down equals R^T e3 */
    quat_t q2 = q_from_euler(0.3f, -0.4f, 1.1f);
    vec3_t d1 = q_body_down(q2), d2 = q_rotate_inv(q2, v3(0, 0, 1));
    CHECK_NEAR(d1.x, d2.x, 1e-6);
    CHECK_NEAR(d1.y, d2.y, 1e-6);
    CHECK_NEAR(d1.z, d2.z, 1e-6);
    CHECK_NEAR(q_tilt_angle(q_from_euler(0.5f, 0, 0.7f)), 0.5, 1e-5);
}

static void test_rotvec(void)
{
    vec3_t rv = v3(0.3f, -0.2f, 0.5f);
    vec3_t back = q_to_rotvec(q_from_rotvec(rv));
    CHECK_NEAR(back.x, rv.x, 1e-6);
    CHECK_NEAR(back.y, rv.y, 1e-6);
    CHECK_NEAR(back.z, rv.z, 1e-6);
    vec3_t tiny = q_to_rotvec(q_from_rotvec(v3(1e-8f, 0, 0)));
    CHECK_NEAR(tiny.x, 1e-8, 1e-9);
    /* composition: two 45 deg rolls = 90 deg roll */
    quat_t a = q_from_rotvec(v3(FM_PI / 4, 0, 0));
    float r;
    q_to_euler(q_mul(a, a), &r, NULL, NULL);
    CHECK_NEAR(r, FM_HALF_PI, 1e-5);
    CHECK_NEAR(q_angle_between(q_identity(), a), FM_PI / 4, 1e-3);
}

int main(void)
{
    RUN(test_trig_vs_libm);
    RUN(test_quat_euler_roundtrip);
    RUN(test_quat_rotation);
    RUN(test_rotvec);
    return TT_SUMMARY();
}

#include "../src/core/fc_math.h"
#include "../src/core/filters.h"
#include "tinytest.h"

/* Measure steady-state sine amplitude through a biquad. */
static double sine_gain(biquad_t *f, double freq, double fs)
{
    biquad_reset(f);
    double peak = 0;
    int n = (int)(fs * 0.5);
    for (int i = 0; i < n; i++) {
        double y = biquad_apply(f, (float)sin(2 * M_PI * freq * i / fs));
        if (i > n / 2 && fabs(y) > peak) peak = fabs(y);
    }
    return peak;
}

static void test_pt1(void)
{
    pt1_t f;
    const float fs = 1000.0f, fc = 10.0f;
    pt1_init(&f, fc, fs);
    /* after one time constant tau = 1/(2 pi fc) a step reaches ~63.2 % */
    int n = (int)(fs / (2 * M_PI * fc) + 0.5);
    float y = 0;
    for (int i = 0; i < n; i++) y = pt1_apply(&f, 1.0f);
    CHECK_NEAR(y, 0.632, 0.02);
    for (int i = 0; i < 2000; i++) y = pt1_apply(&f, 1.0f);
    CHECK_NEAR(y, 1.0, 1e-4);
    pt1_init(&f, 0, fs); /* disabled -> passthrough */
    CHECK_NEAR(pt1_apply(&f, 0.7f), 0.7, 1e-7);
}

static void test_biquad_lpf(void)
{
    biquad_t f;
    const float fs = 8000.0f;
    biquad_lpf_init(&f, 200.0f, fs, BIQUAD_Q_BUTTERWORTH);
    CHECK_NEAR(biquad_gain_at(&f, 1.0f, fs), 1.0, 1e-3);
    CHECK_NEAR(biquad_gain_at(&f, 200.0f, fs), 0.7071, 5e-3);  /* -3 dB */
    CHECK(biquad_gain_at(&f, 800.0f, fs) < 0.07);               /* ~-24 dB */
    /* time-domain check agrees with analytic response */
    CHECK_NEAR(sine_gain(&f, 200.0, fs), biquad_gain_at(&f, 200.0f, fs), 0.01);
    CHECK_NEAR(sine_gain(&f, 50.0, fs), biquad_gain_at(&f, 50.0f, fs), 0.01);
    /* DC step settles to 1 */
    biquad_reset(&f);
    float y = 0;
    for (int i = 0; i < 4000; i++) y = biquad_apply(&f, 1.0f);
    CHECK_NEAR(y, 1.0, 1e-4);
}

static void test_biquad_notch(void)
{
    biquad_t f;
    const float fs = 8000.0f;
    float q = biquad_notch_q(250.0f, 180.0f);
    CHECK(q > 0.5f && q < 2.0f);
    biquad_notch_init(&f, 250.0f, fs, q);
    CHECK(biquad_gain_at(&f, 250.0f, fs) < 1e-3);
    CHECK_NEAR(biquad_gain_at(&f, 180.0f, fs), 0.7071, 0.02);  /* lower edge -3 dB */
    CHECK_NEAR(biquad_gain_at(&f, 5.0f, fs), 1.0, 1e-3);
    CHECK(sine_gain(&f, 250.0, fs) < 0.01);
    /* disabled notch passes through */
    biquad_notch_init(&f, 0.0f, fs, q);
    CHECK(f.passthrough);
    CHECK_NEAR(biquad_apply(&f, 0.3f), 0.3, 1e-7);
}

int main(void)
{
    RUN(test_pt1);
    RUN(test_biquad_lpf);
    RUN(test_biquad_notch);
    return TT_SUMMARY();
}

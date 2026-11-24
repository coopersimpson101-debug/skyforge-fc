/*
 * filters.c - PT1 and biquad filters (RBJ Audio EQ Cookbook formulas).
 */
#include "filters.h"
#include "fc_math.h"

float pt1_gain(float cutoff_hz, float sample_hz)
{
    if (cutoff_hz <= 0.0f || sample_hz <= 0.0f) {
        return 1.0f; /* passthrough */
    }
    const float rc = 1.0f / (FM_TWO_PI * cutoff_hz);
    const float dt = 1.0f / sample_hz;
    return dt / (rc + dt);
}

void pt1_init(pt1_t *f, float cutoff_hz, float sample_hz)
{
    f->k = pt1_gain(cutoff_hz, sample_hz);
    f->y = 0.0f;
}

float pt1_apply(pt1_t *f, float x)
{
    f->y += f->k * (x - f->y);
    return f->y;
}

static void biquad_set_passthrough(biquad_t *f)
{
    f->b0 = 1.0f;
    f->b1 = f->b2 = f->a1 = f->a2 = 0.0f;
    f->passthrough = true;
    biquad_reset(f);
}

void biquad_lpf_init(biquad_t *f, float cutoff_hz, float sample_hz, float q)
{
    if (cutoff_hz <= 0.0f || cutoff_hz >= 0.5f * sample_hz || q <= 0.0f) {
        biquad_set_passthrough(f);
        return;
    }
    const float w0 = FM_TWO_PI * cutoff_hz / sample_hz;
    const float sn = fm_sinf(w0), cs = fm_cosf(w0);
    const float alpha = sn / (2.0f * q);
    const float a0 = 1.0f + alpha;
    f->b0 = ((1.0f - cs) * 0.5f) / a0;
    f->b1 = (1.0f - cs) / a0;
    f->b2 = f->b0;
    f->a1 = (-2.0f * cs) / a0;
    f->a2 = (1.0f - alpha) / a0;
    f->passthrough = false;
    biquad_reset(f);
}

void biquad_notch_init(biquad_t *f, float center_hz, float sample_hz, float q)
{
    if (center_hz <= 0.0f || center_hz >= 0.5f * sample_hz || q <= 0.0f) {
        biquad_set_passthrough(f);
        return;
    }
    const float w0 = FM_TWO_PI * center_hz / sample_hz;
    const float sn = fm_sinf(w0), cs = fm_cosf(w0);
    const float alpha = sn / (2.0f * q);
    const float a0 = 1.0f + alpha;
    f->b0 = 1.0f / a0;
    f->b1 = (-2.0f * cs) / a0;
    f->b2 = f->b0;
    f->a1 = f->b1;
    f->a2 = (1.0f - alpha) / a0;
    f->passthrough = false;
    biquad_reset(f);
}

float biquad_notch_q(float center_hz, float cutoff_hz)
{
    if (cutoff_hz <= 0.0f || cutoff_hz >= center_hz) {
        return 0.0f;
    }
    return center_hz * cutoff_hz / (center_hz * center_hz - cutoff_hz * cutoff_hz);
}

float biquad_apply(biquad_t *f, float x)
{
    const float y = f->b0 * x + f->z1;
    f->z1 = f->b1 * x - f->a1 * y + f->z2;
    f->z2 = f->b2 * x - f->a2 * y;
    return y;
}

void biquad_reset(biquad_t *f)
{
    f->z1 = 0.0f;
    f->z2 = 0.0f;
}

float biquad_gain_at(const biquad_t *f, float f_hz, float sample_hz)
{
    /* H(z) = (b0 + b1 z^-1 + b2 z^-2) / (1 + a1 z^-1 + a2 z^-2), z = e^jw */
    const float w = FM_TWO_PI * f_hz / sample_hz;
    const float c1 = fm_cosf(w), s1 = fm_sinf(w);
    const float c2 = fm_cosf(2.0f * w), s2 = fm_sinf(2.0f * w);
    const float nr = f->b0 + f->b1 * c1 + f->b2 * c2;
    const float ni = -(f->b1 * s1 + f->b2 * s2);
    const float dr = 1.0f + f->a1 * c1 + f->a2 * c2;
    const float di = -(f->a1 * s1 + f->a2 * s2);
    return fm_sqrtf((nr * nr + ni * ni) / (dr * dr + di * di));
}

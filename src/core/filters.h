/*
 * filters.h - PT1 low-pass and RBJ biquad (low-pass / notch) filters.
 */
#ifndef FILTERS_H
#define FILTERS_H

#include <stdbool.h>

/* First-order low-pass:  y += k (x - y),  k = dt / (RC + dt) */
typedef struct {
    float k;
    float y;
} pt1_t;

void  pt1_init(pt1_t *f, float cutoff_hz, float sample_hz);
float pt1_apply(pt1_t *f, float x);
float pt1_gain(float cutoff_hz, float sample_hz);

/* Biquad in transposed direct form II. */
typedef struct {
    float b0, b1, b2, a1, a2; /* normalised: a0 == 1 */
    float z1, z2;             /* state */
    bool  passthrough;        /* true when disabled (cutoff == 0) */
} biquad_t;

#define BIQUAD_Q_BUTTERWORTH 0.70710678f

void  biquad_lpf_init(biquad_t *f, float cutoff_hz, float sample_hz, float q);
void  biquad_notch_init(biquad_t *f, float center_hz, float sample_hz, float q);
/* Q of a notch whose -3 dB lower edge is at cutoff_hz (Betaflight style). */
float biquad_notch_q(float center_hz, float cutoff_hz);
float biquad_apply(biquad_t *f, float x);
void  biquad_reset(biquad_t *f);
/* Magnitude response |H(e^jw)| at frequency f_hz (for tests / docs). */
float biquad_gain_at(const biquad_t *f, float f_hz, float sample_hz);

#endif

/*
 * dshot_tim.c - DShot output on TIM8 CH1..CH4 using a DMA *burst*.
 *
 * TIM8 runs continuously in PWM mode 1 with ARR = one DShot bit period
 * (DShot600 at 168 MHz: 280 ticks = 1.667 us). On every update event the
 * timer raises a DMA request; the DMA burst feature (TIMx_DCR/DMAR) then
 * writes 4 consecutive half-words into CCR1..CCR4, i.e. one bit for all
 * four motors per update. The buffer is interleaved [bit][motor]:
 *
 *     buf = { m1b0, m2b0, m3b0, m4b0,  m1b1, m2b1, ... ,  0,0,0,0, 0,0,0,0 }
 *
 * CCR preload is enabled, so a value written during period k is output in
 * period k+1; the trailing zero slots hold the line low (frame gap).
 * DMA2 Stream1 Channel 7 = TIM8_UP (RM0090 table 43).
 *
 * Bit-bang alternative (for boards whose motor pins do not share a timer):
 * see dshot_bitbang_fill() in src/core/dshot.c and docs/dshot.md - a timer
 * triggers DMA writes of precomputed words into GPIOx->BSRR.
 */
#include "../../core/dshot.h"
#include "periph.h"

#define MOTORS 4
static uint16_t s_buf[DSHOT_DMA_SLOTS * MOTORS] __attribute__((aligned(4)));
static dshot_timing_t s_timing;
static DMA_Stream_TypeDef *const s_dma = DMA2_STREAM(1);

void dshot_tim_init(uint32_t bitrate)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_DMA2EN;
    RCC->APB2ENR |= RCC_APB2ENR_TIM8EN;
    /* PC6..PC9 = TIM8_CH1..4, AF3; pull-down keeps the line low at boot */
    for (int pin = 6; pin <= 9; pin++) {
        gpio_config(GPIOC, pin, GPIO_AF, GPIO_PULLDOWN, 3);
    }
    s_timing = dshot_timing(TIMCLK2_HZ, bitrate);

    TIM8->CR1 = 0;
    TIM8->PSC = 0;
    TIM8->ARR = s_timing.period - 1u;
    /* PWM mode 1 + preload on all four channels */
    const uint32_t oc = (TIM_OCM_PWM1 << 4) | TIM_CCMR_OC1PE;
    TIM8->CCMR1 = oc | (oc << 8);
    TIM8->CCMR2 = oc | (oc << 8);
    TIM8->CCER = (1u << 0) | (1u << 4) | (1u << 8) | (1u << 12); /* CCxE, active high */
    TIM8->CCR1 = TIM8->CCR2 = TIM8->CCR3 = TIM8->CCR4 = 0;
    TIM8->BDTR = TIM_BDTR_MOE;          /* advanced timer: main output enable */
    /* DMA burst: base = CCR1 (offset 0x34 -> 13 words), length 4 transfers */
    TIM8->DCR = (3u << 8) | (0x34u / 4u);
    TIM8->DIER = TIM_DIER_UDE;
    TIM8->CR1 = TIM_CR1_ARPE;
    TIM8->EGR = TIM_EGR_UG;
    TIM8->CR1 |= TIM_CR1_CEN;

    s_dma->CR = 0;
    while (s_dma->CR & DMA_SxCR_EN) {
    }
    s_dma->PAR = (uint32_t)&TIM8->DMAR;
    s_dma->FCR = 0; /* direct mode */
    s_dma->CR = (7u << DMA_SxCR_CHSEL_POS) | DMA_SxCR_PL_HIGH | DMA_SxCR_MSIZE_16 |
                DMA_SxCR_PSIZE_16 | DMA_SxCR_MINC | DMA_SxCR_DIR_M2P;
}

bool dshot_tim_write(const uint16_t packets[4])
{
    if (s_dma->CR & DMA_SxCR_EN) {
        return false; /* previous frame still being sent (~30 us at DShot600) */
    }
    for (int m = 0; m < MOTORS; m++) {
        dshot_fill_ccr(&s_buf[m], MOTORS, packets[m], &s_timing);
    }
    DMA2->LIFCR = DMA_LIFCR_STREAM1_ALL;
    s_dma->M0AR = (uint32_t)s_buf;
    s_dma->NDTR = DSHOT_DMA_SLOTS * MOTORS;
    s_dma->CR |= DMA_SxCR_EN;
    return true;
}

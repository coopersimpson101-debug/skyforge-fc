/*
 * system.c - clock tree, SysTick, DWT cycle counter, reset.
 *
 * Clock tree (HSE = 8 MHz crystal):
 *   PLL input  = HSE / M         = 8 / 8    = 1 MHz   (must be 1..2 MHz)
 *   VCO        = 1 MHz * N       = 1 * 336  = 336 MHz (100..432 MHz)
 *   SYSCLK     = VCO / P         = 336 / 2  = 168 MHz
 *   USB/SDIO   = VCO / Q         = 336 / 7  = 48 MHz
 *   AHB  /1 = 168 MHz, APB1 /4 = 42 MHz (timers 84 MHz),
 *   APB2 /2 = 84 MHz (timers 168 MHz)
 *   Flash: 5 wait states at 168 MHz / 3.3 V, prefetch + I/D cache on.
 * If the crystal fails to start, the same SYSCLK is derived from the
 * 16 MHz HSI (M = 16) and g_hse_ok is cleared (reported by 'status').
 */
#include "periph.h"

volatile uint32_t g_hse_ok;
static volatile uint32_t s_millis;

static void clock_init(void)
{
    /* voltage regulator scale 1 (needed for 168 MHz) */
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    PWR->CR |= PWR_CR_VOS;

    RCC->CR |= RCC_CR_HSEON;
    uint32_t timeout = 500000;
    while (!(RCC->CR & RCC_CR_HSERDY) && --timeout) {
    }
    g_hse_ok = timeout ? 1u : 0u;

    const uint32_t m = g_hse_ok ? 8u : 16u, n = 336u, p_bits = 0u /* /2 */, q = 7u;
    RCC->PLLCFGR = m | (n << 6) | (p_bits << 16) | (q << 24) |
                   (g_hse_ok ? RCC_PLLCFGR_SRC_HSE : 0u);
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY)) {
    }

    FLASH_R->ACR = FLASH_ACR_LATENCY_5WS | FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN;
    RCC->CFGR = RCC_CFGR_PPRE1_DIV4 | RCC_CFGR_PPRE2_DIV2; /* AHB /1 */
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS_MASK) != RCC_CFGR_SWS_PLL) {
    }
}

void system_init(void)
{
    clock_init();
    /* DWT cycle counter for micros() */
    CORE_DEMCR |= (1u << 24); /* TRCENA */
    DWT_LAR = 0xC5ACCE55u;    /* unlock (needed on some parts) */
    DWT_CYCCNT = 0;
    DWT_CTRL |= 1u;
    /* SysTick 1 kHz from HCLK */
    SYSTICK->LOAD = HCLK_HZ / 1000u - 1u;
    SYSTICK->VAL = 0;
    SYSTICK->CTRL = 7u; /* CLKSOURCE = HCLK, TICKINT, ENABLE */
}

void SysTick_Handler(void)
{
    s_millis++;
}

uint32_t millis(void)
{
    return s_millis;
}

uint32_t micros(void)
{
    /* 32-bit cycle counter wraps every 25.6 s at 168 MHz; callers only use
     * differences, so derive a free-running microsecond counter from it. */
    static uint32_t last_cyc, us_acc, cyc_rem;
    uint32_t primask;
    __asm__ volatile("mrs %0, primask\n\tcpsid i" : "=r"(primask)::"memory");
    const uint32_t c = DWT_CYCCNT;
    uint32_t d = c - last_cyc + cyc_rem;
    last_cyc = c;
    us_acc += d / 168u;
    cyc_rem = d % 168u;
    const uint32_t r = us_acc;
    __asm__ volatile("msr primask, %0" ::"r"(primask) : "memory"); /* ISR safe */
    return r;
}

void delay_us(uint32_t us)
{
    const uint32_t start = DWT_CYCCNT;
    const uint32_t cycles = us * (SYSCLK_HZ / 1000000u);
    while ((DWT_CYCCNT - start) < cycles) {
    }
}

void delay_ms(uint32_t ms)
{
    while (ms--) {
        delay_us(1000);
    }
}

void system_reset(void)
{
    dsb();
    SCB_AIRCR = (0x5FAu << 16) | (1u << 2); /* VECTKEY | SYSRESETREQ */
    dsb();
    for (;;) {
    }
}

/*
 * startup.c - vector table and reset handler for STM32F405 (Cortex-M4F).
 *
 * Reset_Handler:
 *   1. optional jump to the ROM DFU bootloader (requested by 'bl' CLI cmd)
 *   2. enable the FPU (CPACR CP10/CP11 full access) before any float code
 *   3. copy .data from flash (LMA) to RAM (VMA), zero .bss
 *   4. system_init(): clocks (168 MHz from 8 MHz HSE), caches, DWT
 *   5. main()
 */
#include <stdint.h>

#include "periph.h"
#include "stm32f4.h"

extern uint32_t _sidata, _sdata, _edata, _sbss, _ebss, _estack;
extern int main(void);

/* survives a software reset (not zeroed: lives in .noinit) */
__attribute__((section(".noinit"))) volatile uint32_t g_boot_magic;
#define BOOT_MAGIC_DFU 0xDF00B007u

void Reset_Handler(void);
void Default_Handler(void);

#define WEAK_ALIAS __attribute__((weak, alias("Default_Handler")))
void NMI_Handler(void) WEAK_ALIAS;
void HardFault_Handler(void) WEAK_ALIAS;
void MemManage_Handler(void) WEAK_ALIAS;
void BusFault_Handler(void) WEAK_ALIAS;
void UsageFault_Handler(void) WEAK_ALIAS;
void SVC_Handler(void) WEAK_ALIAS;
void DebugMon_Handler(void) WEAK_ALIAS;
void PendSV_Handler(void) WEAK_ALIAS;
void SysTick_Handler(void) WEAK_ALIAS;
void EXTI4_IRQHandler(void) WEAK_ALIAS;
void USART1_IRQHandler(void) WEAK_ALIAS;
void USART3_IRQHandler(void) WEAK_ALIAS;
void DMA2_Stream1_IRQHandler(void) WEAK_ALIAS;

typedef void (*vector_t)(void);

__attribute__((section(".isr_vector"), used))
const vector_t g_vectors[16 + STM32F405_IRQ_COUNT] = {
    [0] = (vector_t)&_estack,
    [1] = Reset_Handler,
    [2] = NMI_Handler,
    [3] = HardFault_Handler,
    [4] = MemManage_Handler,
    [5] = BusFault_Handler,
    [6] = UsageFault_Handler,
    [11] = SVC_Handler,
    [12] = DebugMon_Handler,
    [14] = PendSV_Handler,
    [15] = SysTick_Handler,
    [16 + EXTI4_IRQn] = EXTI4_IRQHandler,
    [16 + USART1_IRQn] = USART1_IRQHandler,
    [16 + USART3_IRQn] = USART3_IRQHandler,
    [16 + DMA2_Stream1_IRQn] = DMA2_Stream1_IRQHandler,
};

static void jump_to_system_bootloader(void)
{
    const uint32_t *sys = (const uint32_t *)SYSMEM_BASE_ADDR;
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    SYSCFG->MEMRMP = 1u; /* map system flash at 0x00000000 */
    __asm__ volatile(
        "msr msp, %0\n"
        "bx %1\n" ::"r"(sys[0]), "r"(sys[1]));
    for (;;) {
    }
}

__attribute__((noreturn)) void Reset_Handler(void)
{
    if (g_boot_magic == BOOT_MAGIC_DFU) {
        g_boot_magic = 0;
        jump_to_system_bootloader();
    }
    /* FPU: CP10 + CP11 full access */
    SCB_CPACR |= (0xFu << 20);
    dsb();
    isb();

    uint32_t *src = &_sidata;
    for (uint32_t *dst = &_sdata; dst < &_edata;) {
        *dst++ = *src++;
    }
    for (volatile uint32_t *dst = &_sbss; dst < &_ebss;) {
        *dst++ = 0;
    }
    system_init();
    main();
    for (;;) {
    }
}

void Default_Handler(void)
{
    /* unexpected interrupt / fault: stop the motors by stopping the
     * DShot timer, then hang (watchdog in roadmap) */
    TIM8->CR1 = 0;
    for (;;) {
    }
}

void system_request_bootloader(void)
{
    g_boot_magic = BOOT_MAGIC_DFU;
    system_reset();
}

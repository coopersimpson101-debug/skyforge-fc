/*
 * stm32f4.h - minimal hand-written register map for STM32F405/407.
 *
 * Only the peripherals used by skyforge-fc are described. Offsets and bit
 * positions follow RM0090 (STM32F405/415/407/417 reference manual).
 * No vendor CMSIS / HAL headers are used anywhere in this project.
 */
#ifndef STM32F4_H
#define STM32F4_H

#include <stdint.h>

#define __IO volatile

/* ---- memory map ---------------------------------------------------------- */
#define FLASH_BASE_ADDR   0x08000000u
#define SRAM_BASE_ADDR    0x20000000u
#define CCMRAM_BASE_ADDR  0x10000000u
#define SYSMEM_BASE_ADDR  0x1FFF0000u /* ROM bootloader (USB DFU) */

#define APB1_BASE 0x40000000u
#define APB2_BASE 0x40010000u
#define AHB1_BASE 0x40020000u

/* ---- RCC ----------------------------------------------------------------- */
typedef struct {
    __IO uint32_t CR, PLLCFGR, CFGR, CIR;
    __IO uint32_t AHB1RSTR, AHB2RSTR, AHB3RSTR, RES0;
    __IO uint32_t APB1RSTR, APB2RSTR, RES1[2];
    __IO uint32_t AHB1ENR, AHB2ENR, AHB3ENR, RES2;
    __IO uint32_t APB1ENR, APB2ENR, RES3[2];
    __IO uint32_t AHB1LPENR, AHB2LPENR, AHB3LPENR, RES4;
    __IO uint32_t APB1LPENR, APB2LPENR, RES5[2];
    __IO uint32_t BDCR, CSR, RES6[2];
    __IO uint32_t SSCGR, PLLI2SCFGR;
} RCC_TypeDef;
#define RCC ((RCC_TypeDef *)(AHB1_BASE + 0x3800u))

#define RCC_CR_HSEON      (1u << 16)
#define RCC_CR_HSERDY     (1u << 17)
#define RCC_CR_PLLON      (1u << 24)
#define RCC_CR_PLLRDY     (1u << 25)
#define RCC_PLLCFGR_SRC_HSE (1u << 22)
#define RCC_CFGR_SW_PLL   (2u << 0)
#define RCC_CFGR_SWS_MASK (3u << 2)
#define RCC_CFGR_SWS_PLL  (2u << 2)
#define RCC_CFGR_PPRE1_DIV4 (5u << 10)
#define RCC_CFGR_PPRE2_DIV2 (4u << 13)

#define RCC_AHB1ENR_GPIOAEN (1u << 0)
#define RCC_AHB1ENR_GPIOBEN (1u << 1)
#define RCC_AHB1ENR_GPIOCEN (1u << 2)
#define RCC_AHB1ENR_DMA1EN  (1u << 21)
#define RCC_AHB1ENR_DMA2EN  (1u << 22)
#define RCC_APB1ENR_USART3EN (1u << 18)
#define RCC_APB1ENR_PWREN   (1u << 28)
#define RCC_APB2ENR_TIM8EN  (1u << 1)
#define RCC_APB2ENR_USART1EN (1u << 4)
#define RCC_APB2ENR_SPI1EN  (1u << 12)
#define RCC_APB2ENR_SYSCFGEN (1u << 14)

/* ---- FLASH interface ------------------------------------------------------ */
typedef struct {
    __IO uint32_t ACR, KEYR, OPTKEYR, SR, CR, OPTCR;
} FLASH_TypeDef;
#define FLASH_R ((FLASH_TypeDef *)(AHB1_BASE + 0x3C00u))
#define FLASH_ACR_LATENCY_5WS (5u << 0)
#define FLASH_ACR_PRFTEN  (1u << 8)
#define FLASH_ACR_ICEN    (1u << 9)
#define FLASH_ACR_DCEN    (1u << 10)
#define FLASH_ACR_ICRST   (1u << 11)
#define FLASH_ACR_DCRST   (1u << 12)
#define FLASH_SR_BSY      (1u << 16)
#define FLASH_SR_ERRMASK  (0xF2u) /* OPERR, WRPERR, PGAERR, PGPERR, PGSERR */
#define FLASH_CR_PG       (1u << 0)
#define FLASH_CR_SER      (1u << 1)
#define FLASH_CR_SNB_POS  3u
#define FLASH_CR_PSIZE_X32 (2u << 8)
#define FLASH_CR_STRT     (1u << 16)
#define FLASH_CR_LOCK     (1u << 31)
#define FLASH_KEY1        0x45670123u
#define FLASH_KEY2        0xCDEF89ABu

/* ---- PWR ------------------------------------------------------------------ */
typedef struct {
    __IO uint32_t CR, CSR;
} PWR_TypeDef;
#define PWR ((PWR_TypeDef *)(APB1_BASE + 0x7000u))
#define PWR_CR_VOS (1u << 14)

/* ---- GPIO ------------------------------------------------------------------ */
typedef struct {
    __IO uint32_t MODER, OTYPER, OSPEEDR, PUPDR, IDR, ODR, BSRR, LCKR, AFR[2];
} GPIO_TypeDef;
#define GPIOA ((GPIO_TypeDef *)(AHB1_BASE + 0x0000u))
#define GPIOB ((GPIO_TypeDef *)(AHB1_BASE + 0x0400u))
#define GPIOC ((GPIO_TypeDef *)(AHB1_BASE + 0x0800u))

/* ---- SPI ------------------------------------------------------------------- */
typedef struct {
    __IO uint32_t CR1, CR2, SR, DR, CRCPR, RXCRCR, TXCRCR, I2SCFGR, I2SPR;
} SPI_TypeDef;
#define SPI1 ((SPI_TypeDef *)(APB2_BASE + 0x3000u))
#define SPI_CR1_CPHA (1u << 0)
#define SPI_CR1_CPOL (1u << 1)
#define SPI_CR1_MSTR (1u << 2)
#define SPI_CR1_BR_POS 3u
#define SPI_CR1_SPE  (1u << 6)
#define SPI_CR1_SSI  (1u << 8)
#define SPI_CR1_SSM  (1u << 9)
#define SPI_SR_RXNE  (1u << 0)
#define SPI_SR_TXE   (1u << 1)
#define SPI_SR_BSY   (1u << 7)

/* ---- USART ----------------------------------------------------------------- */
typedef struct {
    __IO uint32_t SR, DR, BRR, CR1, CR2, CR3, GTPR;
} USART_TypeDef;
#define USART1 ((USART_TypeDef *)(APB2_BASE + 0x1000u))
#define USART3 ((USART_TypeDef *)(APB1_BASE + 0x4800u))
#define USART_SR_ORE   (1u << 3)
#define USART_SR_RXNE  (1u << 5)
#define USART_SR_TC    (1u << 6)
#define USART_SR_TXE   (1u << 7)
#define USART_CR1_RE   (1u << 2)
#define USART_CR1_TE   (1u << 3)
#define USART_CR1_RXNEIE (1u << 5)
#define USART_CR1_TXEIE  (1u << 7)
#define USART_CR1_UE   (1u << 13)

/* ---- TIM (advanced/general purpose) -------------------------------------------- */
typedef struct {
    __IO uint32_t CR1, CR2, SMCR, DIER, SR, EGR, CCMR1, CCMR2, CCER, CNT, PSC, ARR, RCR;
    __IO uint32_t CCR1, CCR2, CCR3, CCR4, BDTR, DCR, DMAR;
} TIM_TypeDef;
#define TIM8 ((TIM_TypeDef *)(APB2_BASE + 0x0400u))
#define TIM_CR1_CEN   (1u << 0)
#define TIM_CR1_ARPE  (1u << 7)
#define TIM_DIER_UDE  (1u << 8)
#define TIM_EGR_UG    (1u << 0)
#define TIM_BDTR_MOE  (1u << 15)
#define TIM_OCM_PWM1  6u
#define TIM_CCMR_OC1PE (1u << 3)
#define TIM_CCMR_OC2PE (1u << 11)

/* ---- DMA ----------------------------------------------------------------------- */
typedef struct {
    __IO uint32_t CR, NDTR, PAR, M0AR, M1AR, FCR;
} DMA_Stream_TypeDef;
typedef struct {
    __IO uint32_t LISR, HISR, LIFCR, HIFCR;
} DMA_TypeDef;
#define DMA2 ((DMA_TypeDef *)(AHB1_BASE + 0x6400u))
#define DMA2_STREAM(n) ((DMA_Stream_TypeDef *)(AHB1_BASE + 0x6400u + 0x10u + 0x18u * (n)))
#define DMA_SxCR_EN     (1u << 0)
#define DMA_SxCR_TCIE   (1u << 4)
#define DMA_SxCR_DIR_M2P (1u << 6)
#define DMA_SxCR_MINC   (1u << 10)
#define DMA_SxCR_PSIZE_16 (1u << 11)
#define DMA_SxCR_MSIZE_16 (1u << 13)
#define DMA_SxCR_PL_HIGH (2u << 16)
#define DMA_SxCR_CHSEL_POS 25u
/* stream 1 flags live in LISR/LIFCR bits 6..11 */
#define DMA_LIFCR_STREAM1_ALL (0x3Du << 6)

/* ---- EXTI / SYSCFG --------------------------------------------------------------- */
typedef struct {
    __IO uint32_t IMR, EMR, RTSR, FTSR, SWIER, PR;
} EXTI_TypeDef;
#define EXTI ((EXTI_TypeDef *)(APB2_BASE + 0x3C00u))
typedef struct {
    __IO uint32_t MEMRMP, PMC, EXTICR[4], RES[2], CMPCR;
} SYSCFG_TypeDef;
#define SYSCFG ((SYSCFG_TypeDef *)(APB2_BASE + 0x3800u))

/* ---- Cortex-M4 core ---------------------------------------------------------------- */
typedef struct {
    __IO uint32_t CTRL, LOAD, VAL, CALIB;
} SysTick_TypeDef;
#define SYSTICK ((SysTick_TypeDef *)0xE000E010u)

#define NVIC_ISER ((__IO uint32_t *)0xE000E100u)
#define NVIC_ICER ((__IO uint32_t *)0xE000E180u)
#define NVIC_IPR  ((__IO uint8_t *)0xE000E400u)

#define SCB_VTOR  (*(__IO uint32_t *)0xE000ED08u)
#define SCB_AIRCR (*(__IO uint32_t *)0xE000ED0Cu)
#define SCB_CPACR (*(__IO uint32_t *)0xE000ED88u)
#define DWT_CTRL   (*(__IO uint32_t *)0xE0001000u)
#define DWT_CYCCNT (*(__IO uint32_t *)0xE0001004u)
#define DWT_LAR    (*(__IO uint32_t *)0xE0001FB0u)
#define CORE_DEMCR (*(__IO uint32_t *)0xE000EDFCu)

/* IRQ numbers (STM32F405) */
enum {
    EXTI4_IRQn = 10,
    SPI1_IRQn = 35,
    USART1_IRQn = 37,
    USART3_IRQn = 39,
    TIM8_UP_TIM13_IRQn = 44,
    DMA2_Stream1_IRQn = 57,
    STM32F405_IRQ_COUNT = 82,
};

static inline void nvic_enable(int irq, uint8_t prio)
{
    NVIC_IPR[irq] = (uint8_t)(prio << 4); /* 4 priority bits implemented */
    NVIC_ISER[irq >> 5] = 1u << (irq & 31);
}

static inline void irq_disable(void) { __asm__ volatile("cpsid i" ::: "memory"); }
static inline void irq_enable(void) { __asm__ volatile("cpsie i" ::: "memory"); }
static inline void dsb(void) { __asm__ volatile("dsb" ::: "memory"); }
static inline void isb(void) { __asm__ volatile("isb" ::: "memory"); }

#endif /* STM32F4_H */

/*
 * board.c - generic STM32F405 FPV flight controller.
 *
 * Pin map (modelled on common F405 boards; CHECK YOUR BOARD'S SCHEMATIC
 * and edit this file before flashing - pinouts differ between vendors):
 *
 *   function        pin    peripheral        notes
 *   gyro SCK        PA5    SPI1 AF5
 *   gyro MISO       PA6    SPI1 AF5
 *   gyro MOSI       PA7    SPI1 AF5
 *   gyro CS         PA4    GPIO out          active low
 *   gyro INT        PC4    EXTI4             data ready, rising edge
 *   motor 1..4      PC6..9 TIM8 CH1..4 AF3   DShot, DMA2 Stream1 Ch7
 *   RX (CRSF) TX    PA9    USART1 AF7        420000 baud
 *   RX (CRSF) RX    PA10   USART1 AF7
 *   console TX      PB10   USART3 AF7        115200 baud (CLI)
 *   console RX      PB11   USART3 AF7
 *   status LED      PB9    GPIO out          active high
 */
#include "../board.h"

#include "../../platform/stm32f4/periph.h"

#define GYRO_CS_PORT GPIOA
#define GYRO_CS_PIN  4
#define LED_PORT     GPIOB
#define LED_PIN      9

#define SPI_BR_SLOW 7 /* 84 MHz / 256 = 328 kHz (MPU6000 config <= 1 MHz) */
#define SPI_BR_FAST 2 /* 84 MHz / 8   = 10.5 MHz (sensor reads) */

static uart_t s_rx_uart, s_cli_uart;
static volatile bool s_gyro_ready;

/* ---- interrupt handlers --------------------------------------------------- */
void USART1_IRQHandler(void) { uart_irq(&s_rx_uart); }
void USART3_IRQHandler(void) { uart_irq(&s_cli_uart); }
void EXTI4_IRQHandler(void)
{
    EXTI->PR = 1u << 4; /* write 1 to clear */
    s_gyro_ready = true;
}

/* ---- SPI bus for the IMU driver ---------------------------------------------- */
static int imu_xfer(void *ctx, const uint8_t *tx, uint8_t *rx, int len)
{
    (void)ctx;
    gpio_clear(GYRO_CS_PORT, GYRO_CS_PIN);
    spi_transfer(SPI1, tx, rx, len);
    gpio_set(GYRO_CS_PORT, GYRO_CS_PIN);
    return 0;
}

static void imu_set_fast(void *ctx, bool fast)
{
    (void)ctx;
    spi_set_prescaler(SPI1, fast ? SPI_BR_FAST : SPI_BR_SLOW);
}

static const imu_bus_t s_imu_bus = {imu_xfer, delay_ms, imu_set_fast, 0};

static const config_storage_t s_storage = {flash_erase_config, flash_write_config,
                                           flash_read_config, 0};

void board_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN | RCC_APB2ENR_USART1EN | RCC_APB2ENR_SYSCFGEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
    (void)RCC->APB1ENR; /* read back: clock enable takes effect */

    gpio_config(LED_PORT, LED_PIN, GPIO_OUT, GPIO_NOPULL, 0);

    /* gyro SPI */
    gpio_config(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_OUT, GPIO_PULLUP, 0);
    gpio_set(GYRO_CS_PORT, GYRO_CS_PIN);
    gpio_config(GPIOA, 5, GPIO_AF, GPIO_NOPULL, 5);
    gpio_config(GPIOA, 6, GPIO_AF, GPIO_PULLUP, 5);
    gpio_config(GPIOA, 7, GPIO_AF, GPIO_NOPULL, 5);
    spi_init(SPI1, SPI_BR_SLOW);

    /* gyro data-ready on PC4 -> EXTI4 (enabled after IMU init) */
    gpio_config(GPIOC, 4, GPIO_IN, GPIO_PULLDOWN, 0);
    SYSCFG->EXTICR[1] = (SYSCFG->EXTICR[1] & ~0xFu) | 0x2u; /* EXTI4 <- port C */
    EXTI->RTSR |= 1u << 4;
    EXTI->FTSR &= ~(1u << 4);

    /* UARTs */
    gpio_config(GPIOA, 9, GPIO_AF, GPIO_PULLUP, 7);
    gpio_config(GPIOA, 10, GPIO_AF, GPIO_PULLUP, 7);
    uart_init(&s_rx_uart, USART1, PCLK2_HZ, 420000u, USART1_IRQn);
    gpio_config(GPIOB, 10, GPIO_AF, GPIO_PULLUP, 7);
    gpio_config(GPIOB, 11, GPIO_AF, GPIO_PULLUP, 7);
    uart_init(&s_cli_uart, USART3, PCLK1_HZ, 115200u, USART3_IRQn);
}

const char *board_name(void) { return "F405_GENERIC"; }
uint32_t board_micros(void) { return micros(); }
uint32_t board_millis(void) { return millis(); }
void board_delay_ms(uint32_t ms) { delay_ms(ms); }
const imu_bus_t *board_imu_bus(void) { return &s_imu_bus; }
imu_align_e board_imu_align(void) { return IMU_ALIGN_CW0; }

void board_gyro_irq_enable(void)
{
    EXTI->PR = 1u << 4;
    EXTI->IMR |= 1u << 4;
    nvic_enable(EXTI4_IRQn, 1); /* highest application priority */
}

bool board_gyro_ready(void)
{
    if (!s_gyro_ready) {
        return false;
    }
    s_gyro_ready = false;
    return true;
}

void board_dshot_init(uint32_t bitrate) { dshot_tim_init(bitrate); }
bool board_dshot_write(const uint16_t p[BOARD_MOTORS]) { return dshot_tim_write(p); }

int board_rx_read(uint8_t *b) { return uart_read(&s_rx_uart, b); }
void board_rx_write(const uint8_t *d, int n) { uart_write(&s_rx_uart, d, n); }
int board_cli_read(uint8_t *b) { return uart_read(&s_cli_uart, b); }

void board_cli_write(const char *s)
{
    int n = 0;
    while (s[n]) n++;
    uart_write(&s_cli_uart, (const uint8_t *)s, n);
}

void board_led(bool on)
{
    if (on) gpio_set(LED_PORT, LED_PIN);
    else gpio_clear(LED_PORT, LED_PIN);
}

const config_storage_t *board_config_storage(void) { return &s_storage; }

void board_reboot(bool bootloader)
{
    delay_ms(20); /* let the UART drain */
    if (bootloader) {
        system_request_bootloader();
    }
    system_reset();
}

bool board_clock_ok(void) { return g_hse_ok != 0; }

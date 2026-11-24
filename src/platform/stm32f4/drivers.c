/*
 * drivers.c - GPIO, SPI, UART and internal-flash drivers (register level).
 */
#include "periph.h"

/* ---- GPIO ------------------------------------------------------------------ */
void gpio_config(GPIO_TypeDef *port, int pin, gpio_mode_e mode, gpio_pull_e pull, int af)
{
    const uint32_t p2 = (uint32_t)pin * 2u;
    port->MODER = (port->MODER & ~(3u << p2)) | ((uint32_t)mode << p2);
    port->PUPDR = (port->PUPDR & ~(3u << p2)) | ((uint32_t)pull << p2);
    port->OSPEEDR |= (3u << p2);           /* very high speed */
    port->OTYPER &= ~(1u << pin);          /* push-pull */
    if (mode == GPIO_AF) {
        const int reg = pin >> 3;
        const uint32_t sh = (uint32_t)(pin & 7) * 4u;
        port->AFR[reg] = (port->AFR[reg] & ~(0xFu << sh)) | ((uint32_t)af << sh);
    }
}

/* ---- SPI ------------------------------------------------------------------- */
void spi_set_prescaler(SPI_TypeDef *spi, int br)
{
    while (spi->SR & SPI_SR_BSY) {
    }
    spi->CR1 &= ~SPI_CR1_SPE;
    spi->CR1 = (spi->CR1 & ~(7u << SPI_CR1_BR_POS)) | ((uint32_t)(br & 7) << SPI_CR1_BR_POS);
    spi->CR1 |= SPI_CR1_SPE;
}

void spi_init(SPI_TypeDef *spi, int br)
{
    /* master, mode 3 (CPOL=1, CPHA=1), 8 bit, MSB first, software NSS */
    spi->CR1 = SPI_CR1_MSTR | SPI_CR1_CPOL | SPI_CR1_CPHA | SPI_CR1_SSM | SPI_CR1_SSI |
               ((uint32_t)(br & 7) << SPI_CR1_BR_POS);
    spi->CR2 = 0;
    spi->CR1 |= SPI_CR1_SPE;
}

void spi_transfer(SPI_TypeDef *spi, const uint8_t *tx, uint8_t *rx, int len)
{
    for (int i = 0; i < len; i++) {
        while (!(spi->SR & SPI_SR_TXE)) {
        }
        *(volatile uint8_t *)&spi->DR = tx ? tx[i] : 0xFF;
        while (!(spi->SR & SPI_SR_RXNE)) {
        }
        uint8_t v = *(volatile uint8_t *)&spi->DR;
        if (rx) {
            rx[i] = v;
        }
    }
    while (spi->SR & SPI_SR_BSY) {
    }
}

/* ---- UART ------------------------------------------------------------------ */
void uart_init(uart_t *u, USART_TypeDef *regs, uint32_t pclk, uint32_t baud, int irq)
{
    u->regs = regs;
    u->rx_head = u->rx_tail = 0;
    u->tx_head = u->tx_tail = 0;
    u->overruns = 0;
    regs->CR1 = 0;
    /* oversampling by 16: BRR = fck / baud (mantissa.fraction in 12.4) */
    regs->BRR = (pclk + baud / 2u) / baud;
    regs->CR2 = 0;
    regs->CR3 = 0;
    regs->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;
    nvic_enable(irq, 6);
}

int uart_read(uart_t *u, uint8_t *b)
{
    if (u->rx_head == u->rx_tail) {
        return 0;
    }
    *b = u->rx[u->rx_tail];
    u->rx_tail = (uint16_t)((u->rx_tail + 1u) % UART_RX_BUF);
    return 1;
}

int uart_write(uart_t *u, const uint8_t *d, int n)
{
    int queued = 0;
    for (int i = 0; i < n; i++) {
        uint16_t next = (uint16_t)((u->tx_head + 1u) % UART_TX_BUF);
        if (next == u->tx_tail) {
            break; /* full: drop (telemetry/CLI are best effort) */
        }
        u->tx[u->tx_head] = d[i];
        u->tx_head = next;
        queued++;
    }
    u->regs->CR1 |= USART_CR1_TXEIE;
    return queued;
}

void uart_irq(uart_t *u)
{
    USART_TypeDef *r = u->regs;
    const uint32_t sr = r->SR;
    if (sr & (USART_SR_RXNE | USART_SR_ORE)) {
        const uint8_t b = (uint8_t)r->DR; /* reading DR also clears ORE */
        if (sr & USART_SR_ORE) {
            u->overruns++;
        }
        uint16_t next = (uint16_t)((u->rx_head + 1u) % UART_RX_BUF);
        if (next != u->rx_tail) {
            u->rx[u->rx_head] = b;
            u->rx_head = next;
        }
    }
    if ((r->CR1 & USART_CR1_TXEIE) && (sr & USART_SR_TXE)) {
        if (u->tx_tail != u->tx_head) {
            r->DR = u->tx[u->tx_tail];
            u->tx_tail = (uint16_t)((u->tx_tail + 1u) % UART_TX_BUF);
        } else {
            r->CR1 &= ~USART_CR1_TXEIE;
        }
    }
}

/* ---- internal flash ---------------------------------------------------------- */
static void flash_wait(void)
{
    while (FLASH_R->SR & FLASH_SR_BSY) {
    }
}

static void flash_unlock(void)
{
    if (FLASH_R->CR & FLASH_CR_LOCK) {
        FLASH_R->KEYR = FLASH_KEY1;
        FLASH_R->KEYR = FLASH_KEY2;
    }
}

static void flash_lock(void)
{
    FLASH_R->CR |= FLASH_CR_LOCK;
}

int flash_erase_config(void *ctx)
{
    (void)ctx;
    flash_wait();
    flash_unlock();
    FLASH_R->SR = FLASH_SR_ERRMASK; /* clear stale error flags (write 1) */
    FLASH_R->CR = FLASH_CR_PSIZE_X32 | FLASH_CR_SER | (CONFIG_FLASH_SECTOR << FLASH_CR_SNB_POS);
    FLASH_R->CR |= FLASH_CR_STRT;
    flash_wait();
    const int err = (FLASH_R->SR & FLASH_SR_ERRMASK) ? -1 : 0;
    FLASH_R->CR = 0;
    flash_lock();
    /* invalidate the data cache so reads see the erased sector */
    FLASH_R->ACR &= ~FLASH_ACR_DCEN;
    FLASH_R->ACR |= FLASH_ACR_DCRST;
    FLASH_R->ACR &= ~FLASH_ACR_DCRST;
    FLASH_R->ACR |= FLASH_ACR_DCEN;
    return err;
}

int flash_write_config(void *ctx, uint32_t offset, const void *data, uint32_t len)
{
    (void)ctx;
    if ((offset & 3u) != 0) {
        return -1;
    }
    const uint8_t *src = (const uint8_t *)data;
    flash_wait();
    flash_unlock();
    FLASH_R->CR = FLASH_CR_PSIZE_X32 | FLASH_CR_PG;
    int err = 0;
    for (uint32_t i = 0; i < len; i += 4) {
        uint32_t w = 0xFFFFFFFFu;
        for (uint32_t k = 0; k < 4 && i + k < len; k++) {
            w &= ~(0xFFu << (8u * k));
            w |= (uint32_t)src[i + k] << (8u * k);
        }
        *(volatile uint32_t *)(CONFIG_FLASH_ADDR + offset + i) = w;
        flash_wait();
        if (FLASH_R->SR & FLASH_SR_ERRMASK) {
            err = -1;
            break;
        }
    }
    FLASH_R->CR = 0;
    flash_lock();
    return err;
}

int flash_read_config(void *ctx, uint32_t offset, void *data, uint32_t len)
{
    (void)ctx;
    const volatile uint8_t *p = (const volatile uint8_t *)(CONFIG_FLASH_ADDR + offset);
    uint8_t *d = (uint8_t *)data;
    for (uint32_t i = 0; i < len; i++) {
        d[i] = p[i];
    }
    return 0;
}

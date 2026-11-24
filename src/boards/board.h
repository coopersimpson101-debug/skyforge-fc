/*
 * board.h - board abstraction used by src/app/main.c.
 *
 * Every supported flight controller board implements this interface in
 * src/boards/<board>/board.c on top of its MCU platform layer
 * (src/platform/<mcu>/). The flight code in src/core never touches
 * hardware. Implemented today: f405_generic (STM32F405, see board.c for
 * the pin map). Adding an F722 or H743 board means adding
 * src/platform/stm32f7 or stm32h7 (clock tree, SPI, UART, timer+DMA,
 * flash) and a board directory; nothing in src/core changes.
 */
#ifndef BOARD_H
#define BOARD_H

#include <stdbool.h>
#include <stdint.h>

#include "../core/config.h"
#include "../drivers/imu.h"

#define BOARD_MOTORS 4

void board_init(void);
const char *board_name(void);

uint32_t board_micros(void);
uint32_t board_millis(void);
void board_delay_ms(uint32_t ms);

/* Gyro on SPI with data-ready interrupt. */
const imu_bus_t *board_imu_bus(void);
imu_align_e board_imu_align(void);
/* Returns true once per data-ready interrupt (and clears the flag). */
bool board_gyro_ready(void);
void board_gyro_irq_enable(void);

/* DShot motor outputs. */
void board_dshot_init(uint32_t bitrate);
bool board_dshot_write(const uint16_t packets[BOARD_MOTORS]);

/* Receiver UART (CRSF, 420 kbaud) and console UART (CLI, 115200). */
int  board_rx_read(uint8_t *b);
void board_rx_write(const uint8_t *d, int n);
int  board_cli_read(uint8_t *b);
void board_cli_write(const char *s);

void board_led(bool on);
const config_storage_t *board_config_storage(void);
void board_reboot(bool bootloader);
/* true if the main clock runs from the crystal (false = HSI fallback) */
bool board_clock_ok(void);

#endif

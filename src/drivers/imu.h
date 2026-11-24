/*
 * imu.h - SPI gyro/accelerometer drivers (MPU6000, ICM-42688-P).
 *
 * The drivers are platform independent: all register access goes through
 * an imu_bus_t (full-duplex SPI transfer with chip-select handled by the
 * bus), so they are unit-tested on the host against a register-level
 * mock and run unchanged on the STM32 SPI driver.
 */
#ifndef IMU_H
#define IMU_H

#include <stdbool.h>
#include <stdint.h>
#include "../core/fc_math.h"

typedef struct {
    /* Full-duplex transfer of len bytes with CS asserted for the whole
     * transfer. Returns 0 on success. */
    int (*xfer)(void *ctx, const uint8_t *tx, uint8_t *rx, int len);
    void (*delay_ms)(uint32_t ms);
    /* Optional: switch SPI clock (slow <= 1 MHz for config, fast for data). */
    void (*set_fast)(void *ctx, bool fast);
    void *ctx;
} imu_bus_t;

typedef enum {
    IMU_NONE = 0,
    IMU_MPU6000,
    IMU_ICM42688P,
} imu_type_e;

typedef enum {
    IMU_ALIGN_CW0 = 0,
    IMU_ALIGN_CW90,
    IMU_ALIGN_CW180,
    IMU_ALIGN_CW270,
    IMU_ALIGN_CW0_FLIP,
    IMU_ALIGN_CW90_FLIP,
    IMU_ALIGN_CW180_FLIP,
    IMU_ALIGN_CW270_FLIP,
} imu_align_e;

typedef struct {
    imu_type_e type;
    imu_bus_t bus;
    imu_align_e align;
    float gyro_scale;   /* rad/s per LSB */
    float accel_scale;  /* g per LSB */
    uint32_t odr_hz;
} imu_dev_t;

/* MPU6000 registers */
#define MPU_RA_SMPLRT_DIV     0x19
#define MPU_RA_CONFIG         0x1A
#define MPU_RA_GYRO_CONFIG    0x1B
#define MPU_RA_ACCEL_CONFIG   0x1C
#define MPU_RA_INT_PIN_CFG    0x37
#define MPU_RA_INT_ENABLE     0x38
#define MPU_RA_ACCEL_XOUT_H   0x3B
#define MPU_RA_SIGNAL_PATH_RESET 0x68
#define MPU_RA_USER_CTRL      0x6A
#define MPU_RA_PWR_MGMT_1     0x6B
#define MPU_RA_PWR_MGMT_2     0x6C
#define MPU_RA_WHO_AM_I       0x75
#define MPU6000_WHO_AM_I      0x68

/* ICM-42688-P registers (bank 0) */
#define ICM_RA_DEVICE_CONFIG  0x11
#define ICM_RA_INT_CONFIG     0x14
#define ICM_RA_ACCEL_DATA_X1  0x1F
#define ICM_RA_GYRO_DATA_X1   0x25
#define ICM_RA_PWR_MGMT0      0x4E
#define ICM_RA_GYRO_CONFIG0   0x4F
#define ICM_RA_ACCEL_CONFIG0  0x50
#define ICM_RA_INT_CONFIG1    0x64
#define ICM_RA_INT_SOURCE0    0x65
#define ICM_RA_WHO_AM_I       0x75
#define ICM_RA_BANK_SEL       0x76
#define ICM42688P_WHO_AM_I    0x47

int  imu_read_reg(const imu_bus_t *bus, uint8_t reg, uint8_t *data, int len);
int  imu_write_reg(const imu_bus_t *bus, uint8_t reg, uint8_t value);

/* Probes WHO_AM_I of both supported parts. */
imu_type_e imu_detect(const imu_bus_t *bus);
/* Detect + configure (2000 dps, +-8 g, 8 kHz gyro ODR, data-ready IRQ). */
bool imu_init(imu_dev_t *dev, const imu_bus_t *bus, imu_align_e align);
/* raw[0..2] accel, raw[3..5] gyro, sensor frame. */
bool imu_read_raw(imu_dev_t *dev, int16_t raw[6]);
/* Scaled + aligned to body FRD: gyro rad/s, accel g (specific force). */
bool imu_read(imu_dev_t *dev, vec3_t *gyro, vec3_t *accel);

/* Sensor frame (x fwd, y left, z up when mounted CW0) -> body FRD. */
vec3_t imu_align_vec(vec3_t s, imu_align_e align);
const char *imu_name(imu_type_e t);

#endif

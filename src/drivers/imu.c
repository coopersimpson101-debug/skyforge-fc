/*
 * imu.c - MPU6000 and ICM-42688-P SPI drivers.
 *
 * SPI read: first byte = reg | 0x80, following bytes clock data out.
 * SPI write: first byte = reg & 0x7F, second byte = value.
 * Both parts use SPI mode 3. MPU6000 config registers must be written at
 * <= 1 MHz; sensor registers can be read at up to 20 MHz.
 */
#include "imu.h"

#define GYRO_2000DPS_LSB 16.4f
#define ACC_8G_LSB       4096.0f

int imu_read_reg(const imu_bus_t *bus, uint8_t reg, uint8_t *data, int len)
{
    uint8_t tx[16] = {0};
    uint8_t rx[16] = {0};
    if (len > 15) {
        return -1;
    }
    tx[0] = (uint8_t)(reg | 0x80);
    int r = bus->xfer(bus->ctx, tx, rx, len + 1);
    for (int i = 0; i < len; i++) {
        data[i] = rx[i + 1];
    }
    return r;
}

int imu_write_reg(const imu_bus_t *bus, uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = {(uint8_t)(reg & 0x7F), value};
    uint8_t rx[2];
    return bus->xfer(bus->ctx, tx, rx, 2);
}

static uint8_t read1(const imu_bus_t *bus, uint8_t reg)
{
    uint8_t v = 0;
    imu_read_reg(bus, reg, &v, 1);
    return v;
}

static void set_fast(const imu_bus_t *bus, bool fast)
{
    if (bus->set_fast) {
        bus->set_fast(bus->ctx, fast);
    }
}

imu_type_e imu_detect(const imu_bus_t *bus)
{
    set_fast(bus, false);
    /* ICM-42688-P: make sure bank 0 is selected first (harmless on MPU:
     * 0x76 is an unused register there). */
    imu_write_reg(bus, ICM_RA_BANK_SEL, 0x00);
    for (int attempt = 0; attempt < 3; attempt++) {
        uint8_t who = read1(bus, MPU_RA_WHO_AM_I); /* 0x75 on both parts */
        if (who == ICM42688P_WHO_AM_I) {
            return IMU_ICM42688P;
        }
        if (who == MPU6000_WHO_AM_I) {
            return IMU_MPU6000;
        }
        if (bus->delay_ms) {
            bus->delay_ms(10);
        }
    }
    return IMU_NONE;
}

static bool mpu6000_init(const imu_bus_t *bus)
{
    imu_write_reg(bus, MPU_RA_PWR_MGMT_1, 0x80);        /* device reset   */
    bus->delay_ms(100);
    imu_write_reg(bus, MPU_RA_SIGNAL_PATH_RESET, 0x07); /* gyro/acc/temp  */
    bus->delay_ms(100);
    imu_write_reg(bus, MPU_RA_PWR_MGMT_1, 0x03);        /* clk = PLL gyro Z */
    bus->delay_ms(1);
    imu_write_reg(bus, MPU_RA_USER_CTRL, 0x10);         /* disable I2C    */
    imu_write_reg(bus, MPU_RA_PWR_MGMT_2, 0x00);
    imu_write_reg(bus, MPU_RA_SMPLRT_DIV, 0x00);        /* 8 kHz / (0+1)  */
    imu_write_reg(bus, MPU_RA_CONFIG, 0x00);            /* DLPF 256 Hz -> 8 kHz gyro */
    imu_write_reg(bus, MPU_RA_GYRO_CONFIG, 0x18);       /* +-2000 dps     */
    imu_write_reg(bus, MPU_RA_ACCEL_CONFIG, 0x10);      /* +-8 g          */
    imu_write_reg(bus, MPU_RA_INT_PIN_CFG, 0x10);       /* clear on any read */
    imu_write_reg(bus, MPU_RA_INT_ENABLE, 0x01);        /* data ready IRQ */
    /* verify a couple of writes */
    return read1(bus, MPU_RA_GYRO_CONFIG) == 0x18 &&
           read1(bus, MPU_RA_ACCEL_CONFIG) == 0x10;
}

static bool icm42688_init(const imu_bus_t *bus)
{
    imu_write_reg(bus, ICM_RA_BANK_SEL, 0x00);
    imu_write_reg(bus, ICM_RA_DEVICE_CONFIG, 0x01);     /* soft reset */
    bus->delay_ms(2);
    imu_write_reg(bus, ICM_RA_BANK_SEL, 0x00);
    imu_write_reg(bus, ICM_RA_PWR_MGMT0, 0x0F);         /* gyro + accel low-noise */
    bus->delay_ms(1);                                   /* >= 200 us quiet time */
    imu_write_reg(bus, ICM_RA_GYRO_CONFIG0, 0x03);      /* 2000 dps, 8 kHz */
    imu_write_reg(bus, ICM_RA_ACCEL_CONFIG0, 0x23);     /* 8 g, 8 kHz      */
    imu_write_reg(bus, ICM_RA_INT_CONFIG, 0x03);        /* INT1 push-pull, active high, pulsed */
    imu_write_reg(bus, ICM_RA_INT_CONFIG1, 0x60);       /* 8us pulse, no deassert, async reset off */
    imu_write_reg(bus, ICM_RA_INT_SOURCE0, 0x08);       /* UI data ready -> INT1 */
    bus->delay_ms(50);                                  /* gyro start-up */
    return read1(bus, ICM_RA_GYRO_CONFIG0) == 0x03 &&
           read1(bus, ICM_RA_PWR_MGMT0) == 0x0F;
}

bool imu_init(imu_dev_t *dev, const imu_bus_t *bus, imu_align_e align)
{
    dev->bus = *bus;
    dev->align = align;
    dev->type = imu_detect(bus);
    dev->gyro_scale = (1.0f / GYRO_2000DPS_LSB) * FM_DEG2RAD;
    dev->accel_scale = 1.0f / ACC_8G_LSB;
    dev->odr_hz = 8000;
    bool ok = false;
    switch (dev->type) {
    case IMU_MPU6000:
        ok = mpu6000_init(bus);
        break;
    case IMU_ICM42688P:
        ok = icm42688_init(bus);
        break;
    default:
        ok = false;
        break;
    }
    set_fast(bus, true);
    return ok;
}

static int16_t be16(const uint8_t *p)
{
    return (int16_t)(((uint16_t)p[0] << 8) | p[1]);
}

bool imu_read_raw(imu_dev_t *dev, int16_t raw[6])
{
    uint8_t b[14];
    if (dev->type == IMU_MPU6000) {
        if (imu_read_reg(&dev->bus, MPU_RA_ACCEL_XOUT_H, b, 14) != 0) {
            return false;
        }
        raw[0] = be16(&b[0]);
        raw[1] = be16(&b[2]);
        raw[2] = be16(&b[4]);
        /* b[6..7] = temperature */
        raw[3] = be16(&b[8]);
        raw[4] = be16(&b[10]);
        raw[5] = be16(&b[12]);
        return true;
    }
    if (dev->type == IMU_ICM42688P) {
        if (imu_read_reg(&dev->bus, ICM_RA_ACCEL_DATA_X1, b, 12) != 0) {
            return false;
        }
        for (int i = 0; i < 6; i++) {
            raw[i] = be16(&b[2 * i]);
        }
        return true;
    }
    return false;
}

vec3_t imu_align_vec(vec3_t s, imu_align_e align)
{
    if (align >= IMU_ALIGN_CW0_FLIP) {
        s.y = -s.y;
        s.z = -s.z;
        align = (imu_align_e)(align - IMU_ALIGN_CW0_FLIP);
    }
    switch (align) {
    case IMU_ALIGN_CW90:  return v3(s.y, s.x, -s.z);
    case IMU_ALIGN_CW180: return v3(-s.x, s.y, -s.z);
    case IMU_ALIGN_CW270: return v3(-s.y, -s.x, -s.z);
    case IMU_ALIGN_CW0:
    default:              return v3(s.x, -s.y, -s.z);
    }
}

bool imu_read(imu_dev_t *dev, vec3_t *gyro, vec3_t *accel)
{
    int16_t r[6];
    if (!imu_read_raw(dev, r)) {
        return false;
    }
    vec3_t a = v3(r[0] * dev->accel_scale, r[1] * dev->accel_scale, r[2] * dev->accel_scale);
    vec3_t g = v3(r[3] * dev->gyro_scale, r[4] * dev->gyro_scale, r[5] * dev->gyro_scale);
    /* An accelerometer at rest reads +1 g on the axis pointing up. Since
     * our body frame is FRD (z down), aligning the vector directly gives
     * -1 g on body z at rest, i.e. the specific force in FRD. */
    *accel = imu_align_vec(a, dev->align);
    *gyro = imu_align_vec(g, dev->align);
    return true;
}

const char *imu_name(imu_type_e t)
{
    switch (t) {
    case IMU_MPU6000:   return "MPU6000";
    case IMU_ICM42688P: return "ICM-42688-P";
    default:            return "none";
    }
}

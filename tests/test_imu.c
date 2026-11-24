/*
 * IMU drivers against a register-level SPI mock of MPU6000 / ICM-42688-P.
 */
#include <string.h>

#include "../src/drivers/imu.h"
#include "tinytest.h"

typedef struct {
    uint8_t regs[256];
    uint8_t who;
    int writes;
    uint8_t wlog_reg[64], wlog_val[64];
    bool fast;
    bool present;
} mock_t;

static int mock_xfer(void *ctx, const uint8_t *tx, uint8_t *rx, int len)
{
    mock_t *m = (mock_t *)ctx;
    if (!m->present) {
        memset(rx, 0xFF, (size_t)len); /* MISO pulled up, nobody answers */
        return 0;
    }
    uint8_t reg = tx[0] & 0x7F;
    rx[0] = 0;
    if (tx[0] & 0x80) {
        for (int i = 1; i < len; i++) {
            uint8_t r = (uint8_t)(reg + i - 1);
            rx[i] = (r == 0x75) ? m->who : m->regs[r];
        }
    } else if (len >= 2) {
        m->regs[reg] = tx[1];
        if (m->writes < 64) {
            m->wlog_reg[m->writes] = reg;
            m->wlog_val[m->writes] = tx[1];
        }
        m->writes++;
    }
    return 0;
}

static void mock_delay(uint32_t ms) { (void)ms; }
static void mock_fast(void *ctx, bool fast) { ((mock_t *)ctx)->fast = fast; }

static imu_bus_t bus_for(mock_t *m)
{
    imu_bus_t b = {mock_xfer, mock_delay, mock_fast, m};
    return b;
}

static bool wrote(const mock_t *m, uint8_t reg, uint8_t val)
{
    for (int i = 0; i < m->writes && i < 64; i++) {
        if (m->wlog_reg[i] == reg && m->wlog_val[i] == val) return true;
    }
    return false;
}

static void put_be16(uint8_t *p, int16_t v)
{
    p[0] = (uint8_t)((uint16_t)v >> 8);
    p[1] = (uint8_t)v;
}

static void test_detect(void)
{
    mock_t m;
    memset(&m, 0, sizeof(m));
    m.present = true;
    imu_bus_t b = bus_for(&m);
    m.who = MPU6000_WHO_AM_I;
    CHECK_EQ_INT(imu_detect(&b), IMU_MPU6000);
    m.who = ICM42688P_WHO_AM_I;
    CHECK_EQ_INT(imu_detect(&b), IMU_ICM42688P);
    m.who = 0x12;
    CHECK_EQ_INT(imu_detect(&b), IMU_NONE);
    m.present = false;
    CHECK_EQ_INT(imu_detect(&b), IMU_NONE);
}

static void test_mpu6000_init_and_read(void)
{
    mock_t m;
    memset(&m, 0, sizeof(m));
    m.present = true;
    m.who = MPU6000_WHO_AM_I;
    imu_bus_t b = bus_for(&m);
    imu_dev_t dev;
    CHECK(imu_init(&dev, &b, IMU_ALIGN_CW0));
    CHECK_EQ_INT(dev.type, IMU_MPU6000);
    CHECK(wrote(&m, MPU_RA_PWR_MGMT_1, 0x80));   /* reset */
    CHECK(wrote(&m, MPU_RA_GYRO_CONFIG, 0x18));  /* 2000 dps */
    CHECK(wrote(&m, MPU_RA_ACCEL_CONFIG, 0x10)); /* 8 g */
    CHECK(wrote(&m, MPU_RA_INT_ENABLE, 0x01));   /* data-ready IRQ */
    CHECK(m.fast);                               /* switched to fast SPI */

    /* level at rest: +1 g on sensor z (4096 LSB), roll rate +164 LSB = 10 dps */
    uint8_t *d = &m.regs[MPU_RA_ACCEL_XOUT_H];
    put_be16(d + 0, 0);
    put_be16(d + 2, 0);
    put_be16(d + 4, 4096);
    put_be16(d + 6, 1234); /* temperature, ignored */
    put_be16(d + 8, 164);
    put_be16(d + 10, -328);
    put_be16(d + 12, 82);
    vec3_t g, a;
    CHECK(imu_read(&dev, &g, &a));
    CHECK_NEAR(a.z, -1.0, 1e-6);                 /* FRD specific force */
    CHECK_NEAR(g.x * FM_RAD2DEG, 10.0, 1e-3);
    CHECK_NEAR(g.y * FM_RAD2DEG, 20.0, 1e-3);    /* sensor y (left) -> body y (right) */
    CHECK_NEAR(g.z * FM_RAD2DEG, -5.0, 1e-3);    /* sensor z (up) -> body z (down) */
}

static void test_icm42688_init_and_read(void)
{
    mock_t m;
    memset(&m, 0, sizeof(m));
    m.present = true;
    m.who = ICM42688P_WHO_AM_I;
    imu_bus_t b = bus_for(&m);
    imu_dev_t dev;
    CHECK(imu_init(&dev, &b, IMU_ALIGN_CW90));
    CHECK_EQ_INT(dev.type, IMU_ICM42688P);
    CHECK(wrote(&m, ICM_RA_DEVICE_CONFIG, 0x01)); /* soft reset */
    CHECK(wrote(&m, ICM_RA_PWR_MGMT0, 0x0F));     /* LN mode */
    CHECK(wrote(&m, ICM_RA_GYRO_CONFIG0, 0x03));  /* 2000 dps 8 kHz */
    CHECK(wrote(&m, ICM_RA_INT_SOURCE0, 0x08));   /* DRDY -> INT1 */

    uint8_t *d = &m.regs[ICM_RA_ACCEL_DATA_X1];
    put_be16(d + 0, 2048);  /* ax 0.5 g */
    put_be16(d + 2, 0);
    put_be16(d + 4, 3547);  /* az 0.866 g */
    put_be16(d + 6, 164);   /* gx 10 dps */
    put_be16(d + 8, 0);
    put_be16(d + 10, 0);
    vec3_t g, a;
    CHECK(imu_read(&dev, &g, &a));
    /* CW90: body = (sy, sx, -sz) */
    CHECK_NEAR(a.x, 0.0, 1e-6);
    CHECK_NEAR(a.y, 0.5, 1e-6);
    CHECK_NEAR(a.z, -3547.0 / 4096.0, 1e-6);
    CHECK_NEAR(g.y * FM_RAD2DEG, 10.0, 1e-3);
}

static void test_alignment_is_rotation(void)
{
    /* every alignment must preserve length and handedness */
    for (int al = 0; al < 8; al++) {
        vec3_t x = imu_align_vec(v3(1, 0, 0), (imu_align_e)al);
        vec3_t y = imu_align_vec(v3(0, 1, 0), (imu_align_e)al);
        vec3_t z = imu_align_vec(v3(0, 0, 1), (imu_align_e)al);
        vec3_t c = v3_cross(x, y);
        /* sensor frame right-handed (FLU) -> body FRD right-handed */
        CHECK_NEAR(c.x, z.x, 1e-6);
        CHECK_NEAR(c.y, z.y, 1e-6);
        CHECK_NEAR(c.z, z.z, 1e-6);
        CHECK_NEAR(v3_norm(x) * v3_norm(y), 1.0, 1e-6);
    }
}

int main(void)
{
    RUN(test_detect);
    RUN(test_mpu6000_init_and_read);
    RUN(test_icm42688_init_and_read);
    RUN(test_alignment_is_rotation);
    return TT_SUMMARY();
}

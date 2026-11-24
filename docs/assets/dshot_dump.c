/*
 * dshot_dump.c - print what the real DShot encoder (src/core/dshot.c) emits
 * for one motor command, for docs/assets/make_figures.py.
 *
 *   dshot_dump <motor 0..1> <idle 0..0.5> <timer_hz> <bitrate>
 *
 * Output (one key=value per line): value, packet, crc, period/t0h/t1h ticks
 * and the 18 CCR slots written to the DMA buffer.
 */
#include <stdio.h>
#include <stdlib.h>

#include "dshot.h"

int main(int argc, char **argv)
{
    if (argc != 5) {
        fprintf(stderr, "usage: %s motor idle timer_hz bitrate\n", argv[0]);
        return 2;
    }
    float motor = (float)atof(argv[1]);
    float idle = (float)atof(argv[2]);
    uint32_t clk = (uint32_t)strtoul(argv[3], NULL, 10);
    uint32_t rate = (uint32_t)strtoul(argv[4], NULL, 10);

    uint16_t value = dshot_value_from_motor(motor, idle);
    uint16_t packet = dshot_encode(value, false, false);
    dshot_timing_t t = dshot_timing(clk, rate);
    uint16_t ccr[DSHOT_DMA_SLOTS];
    dshot_fill_ccr(ccr, 1, packet, &t);

    printf("value=%u\npacket=0x%04X\ncrc=%u\n", value, packet, packet & 0xFu);
    printf("period=%lu\nt0h=%lu\nt1h=%lu\nccr=", (unsigned long)t.period, (unsigned long)t.t0h,
           (unsigned long)t.t1h);
    for (int i = 0; i < DSHOT_DMA_SLOTS; i++) printf("%s%u", i ? "," : "", ccr[i]);
    printf("\n");
    return 0;
}

# Protocols: DShot and CRSF

## DShot (FC -> ESC)

### Frame

```
 bit: 15 ............ 5   4    3 .. 0
      [ 11-bit value  ] [T] [ CRC4  ]

 v   = value << 1 | T                   (12 bits)
 crc = (v ^ v>>4 ^ v>>8) & 0xF          (XOR of the three nibbles)
 bidirectional DShot: crc = ~crc & 0xF  (and the line is inverted)
```

| value     | meaning |
|-----------|---------|
| 0         | motor stop / disarmed |
| 1 .. 47   | special commands, sent with T = 1 (beeps, spin direction, 3D mode, save, LEDs, ...) |
| 48 .. 2047| throttle, 2000 steps |

Commands that change ESC settings (spin direction, 3D mode, save) are only
accepted after repeated frames; `dshot_cmd_repeats()` sends them 10 times
(spec minimum 6) followed by a gap of `MOTOR_STOP` frames. The command queue
(`dshot_cmdq_*`) runs only while disarmed.

Reference vectors in `tests/test_dshot.c` come from an independent
bit-string implementation (`tools/gen_reference_vectors.py`), e.g.
`1046, T=0 -> 0x82C6`.

### Bit timing

| rate     | bit time | '0' high (37.5 %) | '1' high (75 %) |
|----------|----------|-------------------|-----------------|
| DShot150 | 6.67 us  | 2.50 us           | 5.00 us         |
| DShot300 | 3.33 us  | 1.25 us           | 2.50 us         |
| DShot600 | 1.67 us  | 0.625 us          | 1.25 us         |

At the 168 MHz TIM8 clock, DShot600 is ARR+1 = 280 ticks, CCR = 105 for a
'0' and 210 for a '1' (`dshot_timing()`, checked in the tests).

### Timer + DMA burst (implemented, `src/platform/stm32f4/dshot_tim.c`)

All four motor pins sit on one advanced timer (TIM8 CH1..4 on PC6..PC9).
TIM8 runs continuously in PWM mode 1. Each update event issues a DMA
request; with the DMA *burst* feature (`TIMx_DCR`: base = CCR1, length = 4)
one request transfers four half-words through `TIMx_DMAR` into CCR1..CCR4.
So one DMA stream feeds all four motors, one bit per update:

```
buffer (uint16, interleaved):  m1b0 m2b0 m3b0 m4b0 | m1b1 ... | m1b15 .. m4b15 | 0 0 0 0 | 0 0 0 0
                               '-- one update event (burst of 4) --'                   frame gap
```

CCR preload is on, so a value written during period k is output in period
k+1; the trailing zero slots hold the lines low between frames. A frame takes
18 x 1.67 us = 30 us. DMA2 Stream1 Channel 7 is TIM8_UP on the F405.

### GPIO bit-bang alternative (encoder implemented, driver = roadmap)

When motor pins do not share a timer, a timer can trigger DMA writes of
precomputed 32-bit words to `GPIOx->BSRR` instead. `dshot_bitbang_fill()`
splits each bit into 8 slots: slot 0 sets every motor pin, slot 3 resets the
pins sending '0' (37.5 %), slot 6 resets the pins sending '1' (75 %). One
DMA stream then drives up to 16 pins of one port. The encoder is unit tested;
the timer/DMA plumbing for it is not written yet.

### Bidirectional DShot (roadmap)

Needs: inverted line, inverted CRC (already supported by `dshot_encode(...,
bidir=true)`), switching the pin to input after each frame and capturing the
21-bit GCR eRPM reply with timer input capture + DMA, then GCR decoding.
This feeds RPM notch filters.

## CRSF / ExpressLRS (receiver -> FC)

UART 420000 baud, 8N1, not inverted. Frame:

```
[sync 0xC8][len][type][payload ...][crc8]
len  = 1 (type) + payload + 1 (crc)          (2 .. 62)
crc8 = CRC-8/DVB-S2, poly 0xD5, init 0, over type + payload
```

The parser (`crsf_parse_byte`) is a byte-wise state machine: it waits for a
sync/address byte, validates the length, collects the frame, checks the CRC,
and resets on an inter-byte gap > 1.5 ms. Errors are counted
(`crc_errors`, `framing_errors`).

`RC_CHANNELS_PACKED (0x16)`: 22 bytes = 16 channels x 11 bits, LSB first.
172 -> 988 us, 992 -> 1500 us, 1811 -> 2012 us
(`us = (v - 992) * 5/8 + 1500`). Channel order AETR: roll, pitch, throttle,
yaw, then AUX1 = arm (`arm_channel`, > 1700 us), AUX2 = mode
(`mode_channel`, > 1300 us = angle).

`LINK_STATISTICS (0x14)` is decoded (RSSI, LQ, SNR, RF mode, TX power).

Telemetry sent back at 10 Hz: `ATTITUDE (0x1E)` (pitch/roll/yaw in
1e-4 rad, big endian) and `FLIGHT_MODE (0x21)` ("ACRO", "ANGL", "!FS!", `*`
suffix when disarmed). `crsf_build_battery()` exists but there is no
battery ADC driver yet.

The test frame bytes in `tests/test_crsf.c` were produced by the
independent packer in `tools/gen_reference_vectors.py`; the CRC is also
checked against the catalogue check value `crc8("123456789") = 0xBC`.

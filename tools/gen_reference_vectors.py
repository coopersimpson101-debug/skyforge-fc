#!/usr/bin/env python3
"""
gen_reference_vectors.py - independent reference implementations used to
produce the hard-coded vectors in tests/test_dshot.c and tests/test_crsf.c.

They deliberately work on bit *strings* rather than shifts/masks so they do
not share structure (or bugs) with the C code under test.

    python tools/gen_reference_vectors.py
"""


def dshot(value, telemetry, bidir=False):
    bits = format(value, "011b") + ("1" if telemetry else "0")
    nibbles = [int(bits[i:i + 4], 2) for i in (0, 4, 8)]
    crc = nibbles[0] ^ nibbles[1] ^ nibbles[2]
    if bidir:
        crc = (~crc) & 0xF
    return int(bits + format(crc, "04b"), 2)


def crc8_dvb_s2(data):
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0xD5) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def crsf_rc_frame(channels):
    # 16 x 11 bit, least significant bit first
    bits = "".join(format(c, "011b")[::-1] for c in channels)
    payload = bytes(int(bits[i:i + 8][::-1], 2) for i in range(0, len(bits), 8))
    frame = bytes([0xC8, len(payload) + 2, 0x16]) + payload
    return frame + bytes([crc8_dvb_s2(frame[2:])])


def main():
    print("DShot (value, telemetry) -> packet, bidirectional packet")
    for v, t in [(1046, 0), (48, 0), (0, 0), (2047, 0), (1000, 1), (21, 1), (7, 1), (1500, 0)]:
        print(f"  {v:4d} {t}  0x{dshot(v, t):04X}  0x{dshot(v, t, True):04X}")
    print(f"CRC-8/DVB-S2('123456789') = 0x{crc8_dvb_s2(b'123456789'):02X}  (catalogue: 0xBC)")
    ch = [172, 992, 1811, 1234, 191, 1792, 992, 992, 0, 2047, 1, 1024, 500, 1500, 1700, 999]
    f = crsf_rc_frame(ch)
    print(f"CRSF RC frame ({len(f)} bytes):")
    print("  " + ", ".join(f"0x{b:02X}" for b in f))
    ls = bytes([0xC8, 12, 0x14, 60, 70, 100, 10, 1, 4, 3, 55, 98, 8])
    ls += bytes([crc8_dvb_s2(ls[2:])])
    print("CRSF link statistics frame:")
    print("  " + ", ".join(f"0x{b:02X}" for b in ls))


if __name__ == "__main__":
    main()

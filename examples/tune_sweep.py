#!/usr/bin/env python3
"""
tune_sweep.py - grid-search roll P/D in the SIMULATOR and print the
step-response metrics. A starting point for understanding the gains, not a
substitute for tuning on the real quad (the sim has no prop wash, no frame
resonances and an idealised motor model).

    python tools/build.py sim
    python examples/tune_sweep.py
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "sim"))
from skyforge_sim import Sim  # noqa: E402


def step_metrics(p, d):
    s = Sim()
    s.set_param("p_roll", p)
    s.set_param("d_roll", d)
    s.arm()
    hov = s.hover_throttle()
    s.rc(throttle=hov + 0.08)
    s.run(1.0, log_hz=100)
    s.rc(throttle=hov)
    s.run(1.0, log_hz=100)
    s.rc(roll=0.6)
    log = s.run(0.4)
    t0 = next(r["t"] for r in log if r["sp_p"] > 1.0)
    sp = max(r["sp_p"] for r in log)
    peak = max(r["p"] for r in log)
    t90 = next((r["t"] - t0 for r in log if r["p"] >= 0.9 * sp), float("nan"))
    tail = [abs(r["p"] - r["sp_p"]) for r in log if r["t"] > t0 + 0.15]
    return 1000 * t90, 100 * (peak - sp) / sp, sum(tail) / len(tail)


def main():
    print(" P    D   | t90 [ms]  overshoot [%]  mean|err| after 150 ms [deg/s]   (SIMULATION)")
    for p in (40, 55, 70):
        for d in (30, 45, 60):
            t90, ov, err = step_metrics(p, d)
            print(f"{p:3d}  {d:3d}   | {t90:7.1f}   {ov:12.1f}   {err:10.1f}")


if __name__ == "__main__":
    main()

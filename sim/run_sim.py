#!/usr/bin/env python3
"""
run_sim.py - fly the simulator and plot / summarise the response.

    python sim/run_sim.py step  [--param p_roll=60 ...] [--plot out.png] [--csv out.csv]
    python sim/run_sim.py gust  [...]
    python sim/run_sim.py failsafe

Plots need matplotlib (optional); without it a text summary is printed.
All numbers come from the SIMULATOR (sim/quad_sim.c), not from a real quad.
"""
import argparse
import csv
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from skyforge_sim import Sim  # noqa: E402


def scenario_step(s):
    s.arm(angle_mode=False)
    hov = s.hover_throttle()
    log = []
    s.rc(throttle=hov + 0.08)
    log += s.run(1.0)
    s.rc(throttle=hov)
    log += s.run(1.0)
    s.rc(roll=0.6)
    log += s.run(0.4)
    s.rc(roll=0.0)
    log += s.run(0.4)
    return log, "acro roll-rate step (stick 0.6)", ("p", "sp_p", "roll rate [deg/s]")


def scenario_gust(s):
    s.arm(angle_mode=True)
    hov = s.hover_throttle()
    log = []
    s.rc(throttle=hov + 0.08)
    log += s.run(1.0)
    s.rc(throttle=hov)
    log += s.run(1.0)
    s.disturbance(0.6, -0.25, 0.0)
    log += s.run(0.15)
    s.disturbance(0, 0, 0)
    log += s.run(1.5)
    return log, "angle mode: 150 ms torque gust recovery", ("roll", "est_roll", "roll [deg]")


def scenario_failsafe(s):
    s.set_param("fs_land_time_s", 4.0)
    s.arm(angle_mode=False)
    hov = s.hover_throttle()
    log = []
    s.rc(throttle=hov + 0.15)
    log += s.run(1.5)
    s.rc(throttle=hov)
    log += s.run(0.5)
    s.link(False)
    log += s.run(5.0)
    return log, "failsafe: link lost at t=3.5 s", ("alt", "armed", "altitude [m] / armed")


SCENARIOS = {"step": scenario_step, "gust": scenario_gust, "failsafe": scenario_failsafe}


def summarise(name, log):
    if name == "step":
        t0 = next(r["t"] for r in log if r["sp_p"] > 1.0)
        win = [r for r in log if t0 <= r["t"] <= t0 + 0.4]
        sp = max(r["sp_p"] for r in win)
        peak = max(r["p"] for r in win)
        t90 = next((r["t"] - t0 for r in win if r["p"] >= 0.9 * sp), float("nan"))
        print(f"setpoint {sp:.0f} deg/s, t90 {1000 * t90:.1f} ms, overshoot {100 * (peak - sp) / sp:.1f} %")
    elif name == "gust":
        peak = max(abs(r["roll"]) for r in log)
        print(f"peak roll {peak:.1f} deg, final roll {log[-1]['roll']:.2f} deg, "
              f"pitch {log[-1]['pitch']:.2f} deg")
    else:
        lost = next(r["t"] for r in log if r["fs"] >= 1)
        dis = next((r["t"] for r in log if r["t"] > lost and r["armed"] < 0.5), float("nan"))
        print(f"failsafe at t={lost:.2f} s, disarmed at t={dis:.2f} s, final alt {log[-1]['alt']:.2f} m")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("scenario", choices=sorted(SCENARIOS))
    ap.add_argument("--param", action="append", default=[], help="name=value (CLI parameter names)")
    ap.add_argument("--plot", help="write a PNG plot (needs matplotlib)")
    ap.add_argument("--csv", help="write the log as CSV")
    a = ap.parse_args()

    s = Sim()
    for kv in a.param:
        k, v = kv.split("=", 1)
        s.set_param(k, float(v))
    log, title, (y1, y2, ylabel) = SCENARIOS[a.scenario](s)
    summarise(a.scenario, log)

    if a.csv:
        with open(a.csv, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(log[0].keys()))
            w.writeheader()
            w.writerows(log)
    if a.plot:
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
        except ImportError:
            sys.exit("matplotlib not installed: pip install matplotlib")
        t = [r["t"] for r in log]
        fig, ax = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
        ax[0].plot(t, [r[y1] for r in log], label=y1)
        ax[0].plot(t, [r[y2] for r in log], "--", label=y2)
        ax[0].set_ylabel(ylabel)
        ax[0].legend()
        ax[0].set_title(f"skyforge-fc SIMULATION - {title}")
        for m in ("m1", "m2", "m3", "m4"):
            ax[1].plot(t, [r[m] for r in log], label=m)
        ax[1].set_ylabel("motor command")
        ax[1].set_xlabel("time [s]")
        ax[1].legend(ncol=4)
        fig.tight_layout()
        fig.savefig(a.plot, dpi=110)
        print(f"wrote {a.plot}")


if __name__ == "__main__":
    main()

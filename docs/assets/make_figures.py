#!/usr/bin/env python3
"""Regenerate the README images in docs/assets/ from the real flight code.

    pip install ziglang matplotlib          # or set $CC to a host C compiler
    python tools/build.py sim               # builds build/host/skyforge_sim.(dll|so)
    python docs/assets/make_figures.py

Every curve comes from the closed-loop simulator (sim/quad_sim.c driving the real
src/core flight code through sim/skyforge_sim.py). The DShot diagram is drawn from
what src/core/dshot.c actually emits (docs/assets/dshot_dump.c). Nothing here comes
from a real quad.

Outputs:
    banner.svg      README banner; motif = simulated roll-rate step + DShot600 frame
    sim_hero.png    roll-rate step, gust recovery (Madgwick vs EKF), motor outputs
    dshot600.svg    bit timing of one DShot600 frame for a sample motor command
    sim_terminal.svg  captured stdout of build/host/sim_main step / gust
"""
from __future__ import annotations

import importlib.util
import math
import os
import subprocess
import sys
from xml.sax.saxutils import escape

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(ROOT, "sim"))

from skyforge_sim import Sim  # noqa: E402

ACCENT = "#EF4444"
BG, FG, MUTED, GRIDC, EDGE = "#0d1117", "#e6edf3", "#8b949e", "#161b22", "#30363d"
BLUE, GREEN, AMBER = "#58a6ff", "#3fb950", "#d29922"
MONO = "ui-monospace, SFMono-Regular, Menlo, Consolas, monospace"
SAMPLE_MOTOR, IDLE = 0.5, 0.055  # motor command and default motor_idle (src/core/config.c)


def write(name: str, text: str) -> None:
    path = os.path.join(HERE, name)
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        f.write(text)
    print(f"wrote {name} ({os.path.getsize(path) / 1024:.1f} KiB)")


# ------------------------------------------------------------------ simulator runs
def hover_then(s: Sim, angle: bool):
    assert s.arm(angle_mode=angle), "arming failed in the simulator"
    hov = s.hover_throttle()
    log = []
    s.rc(throttle=hov + 0.08)
    log += s.run(1.0)
    s.rc(throttle=hov)
    log += s.run(1.0)
    return log


def run_step():
    """Same scenario as `python sim/run_sim.py step`."""
    s = Sim()
    log = hover_then(s, angle=False)
    s.rc(roll=0.6)
    log += s.run(0.4)
    s.rc(roll=0.0)
    log += s.run(0.4)
    return log


def run_gust(ahrs_type: int):
    """Same scenario as `python sim/run_sim.py gust` (ahrs_type 0 = Madgwick, 1 = EKF)."""
    s = Sim()
    s.set_param("ahrs_type", ahrs_type)
    log = hover_then(s, angle=True)
    s.gust_t = log[-1]["t"]
    s.disturbance(0.6, -0.25, 0.0)
    log += s.run(0.15)
    s.disturbance(0, 0, 0)
    log += s.run(1.5)
    return log, s.gust_t


def step_metrics(log):
    t0 = next(r["t"] for r in log if r["sp_p"] > 1.0)
    win = [r for r in log if t0 <= r["t"] <= t0 + 0.4]
    sp = max(r["sp_p"] for r in win)
    peak = max(r["p"] for r in win)
    t90 = next(r["t"] - t0 for r in win if r["p"] >= 0.9 * sp)
    return t0, sp, t90, 100 * (peak - sp) / sp


# ------------------------------------------------------------------ hero plot
def hero_png(step, gust_m, gust_e, path):
    (gm, tg), (ge, tge) = gust_m, gust_e
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({"font.family": "monospace", "font.size": 9})
    t0, sp, t90, ovs = step_metrics(step)
    fig, ax = plt.subplots(1, 3, figsize=(13, 3.9), dpi=110)
    fig.patch.set_facecolor(BG)
    for a in ax:
        a.set_facecolor(BG)
        a.tick_params(colors=MUTED)
        a.grid(color=GRIDC)
        for sp_ in a.spines.values():
            sp_.set_color(EDGE)
    # (a) roll-rate step
    w = [r for r in step if t0 - 0.05 <= r["t"] <= t0 + 0.55]
    tt = [(r["t"] - t0) * 1000 for r in w]
    ax[0].plot(tt, [r["sp_p"] for r in w], "--", color=MUTED, lw=1.4, label="setpoint")
    ax[0].plot(tt, [r["p"] for r in w], color=ACCENT, lw=2, label="roll rate (true)")
    ax[0].axvline(t90 * 1000, color=EDGE, lw=1)
    ax[0].annotate(f"t90 {t90 * 1000:.0f} ms\novershoot {ovs:.0f} %", (t90 * 1000, sp * 0.45),
                   xytext=(8, 0), textcoords="offset points", color=FG, fontsize=8)
    ax[0].set_title("acro roll-rate step (stick 0.6)", color=FG)
    ax[0].set_xlabel("time after step [ms]", color=MUTED)
    ax[0].set_ylabel("deg/s", color=MUTED)
    # (b) gust recovery
    for log, t_g, col, name in ((gm, tg, ACCENT, "Madgwick"), (ge, tge, BLUE, "EKF")):
        w = [r for r in log if r["t"] >= t_g - 0.2]
        ax[1].plot([r["t"] - t_g for r in w], [r["roll"] for r in w], color=col, lw=2, label=f"{name}: true roll")
        ax[1].plot([r["t"] - t_g for r in w], [r["est_roll"] for r in w], ":", color=col, lw=1.3,
                   label=f"{name}: estimate")
    ax[1].axvspan(0, 0.15, color=AMBER, alpha=0.15, lw=0)
    ax[1].text(0.3, ax[1].get_ylim()[1] * 0.85, "gust: 0.6 + 0.25 N·m, 150 ms", color=AMBER, fontsize=8)
    ax[1].set_title("angle mode: gust recovery", color=FG)
    ax[1].set_xlabel("time after gust [s]", color=MUTED)
    ax[1].set_ylabel("roll [deg]", color=MUTED)
    # (c) motor outputs during the gust (Madgwick)
    w = [r for r in gm if r["t"] >= tg - 0.1 and r["t"] <= tg + 0.6]
    for m, col in zip(("m1", "m2", "m3", "m4"), (ACCENT, BLUE, GREEN, AMBER)):
        ax[2].plot([r["t"] - tg for r in w], [r[m] for r in w], color=col, lw=1.6, label=m)
    ax[2].axvspan(0, 0.15, color=AMBER, alpha=0.15, lw=0)
    ax[2].set_title("mixer outputs during the gust", color=FG)
    ax[2].set_xlabel("time after gust [s]", color=MUTED)
    ax[2].set_ylabel("motor command [0..1]", color=MUTED)
    for a in ax:
        a.legend(facecolor=BG, edgecolor=EDGE, labelcolor=FG, fontsize=7.5)
    fig.suptitle("SIMULATION: real src/core flight code in the closed-loop rigid-body model (sim/quad_sim.c)",
                 color=MUTED, fontsize=9, y=0.995)
    fig.tight_layout()
    fig.savefig(path, facecolor=BG)
    print(f"wrote {os.path.basename(path)} ({os.path.getsize(path) / 1024:.1f} KiB)")
    return t90, ovs, max(abs(r["roll"]) for r in gm), max(abs(r["roll"]) for r in ge)


# ------------------------------------------------------------------ DShot from the real encoder
def load_build_py():
    spec = importlib.util.spec_from_file_location("skyforge_build", os.path.join(ROOT, "tools", "build.py"))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def dshot_dump(motor=SAMPLE_MOTOR, idle=IDLE, clk=168_000_000, rate=600_000):
    b = load_build_py()
    out_dir = os.path.join(ROOT, "build", "host")
    os.makedirs(out_dir, exist_ok=True)
    exe = os.path.join(out_dir, "dshot_dump" + (".exe" if os.name == "nt" else ""))
    subprocess.run(b.host_cc() + ["-std=c11", "-O2", "-Wall", "-Wextra", "-Werror", "-DFC_HOST",
                                  "-I" + os.path.join(ROOT, "src", "core"), "-o", exe,
                                  os.path.join(HERE, "dshot_dump.c"),
                                  os.path.join(ROOT, "src", "core", "dshot.c")], check=True)
    txt = subprocess.run([exe, str(motor), str(idle), str(clk), str(rate)], check=True,
                         capture_output=True, text=True).stdout
    d = dict(ln.split("=", 1) for ln in txt.split())
    d = {k: (v if k in ("packet", "ccr") else int(v)) for k, v in d.items()}
    d["ccr"] = [int(x) for x in d["ccr"].split(",")]
    d["packet_int"] = int(d["packet"], 16)
    d.update(motor=motor, idle=idle, clk=clk, rate=rate)
    print(f"dshot: motor {motor} -> value {d['value']} packet {d['packet']} ccr {d['ccr'][:4]}...")
    return d


def dshot_svg(d) -> str:
    W, H = 1200, 330
    x0, bw, yhi, ylo = 40, 66, 120, 200
    per, ns_tick = d["period"], 1e9 / d["clk"]
    bit_ns = per * ns_tick
    bits = [(d["packet_int"] >> (15 - i)) & 1 for i in range(16)]
    o = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}" '
         f'font-family="{MONO}">', f'<rect width="{W}" height="{H}" rx="10" fill="{BG}"/>',
         f'<text x="{x0}" y="36" fill="{FG}" font-size="17" font-weight="bold">DShot600 frame for motor '
         f'command {d["motor"]} (idle {d["idle"] * 100:.1f} %) → throttle value {d["value"]} → packet '
         f'{d["packet"]}</text>',
         f'<text x="{x0}" y="60" fill="{MUTED}" font-size="13">TIM8 @ {d["clk"] / 1e6:.0f} MHz: '
         f'{per} ticks/bit = {bit_ns:.0f} ns · "0" = CCR {d["t0h"]} ({d["t0h"] * ns_tick:.0f} ns high) · '
         f'"1" = CCR {d["t1h"]} ({d["t1h"] * ns_tick:.0f} ns high) · frame {16 * bit_ns / 1000:.2f} µs</text>']
    # field brackets
    fields = [("11-bit throttle value", 0, 11, ACCENT), ("T", 11, 12, AMBER), ("CRC4", 12, 16, BLUE)]
    for name, a, b, col in fields:
        xa, xb = x0 + a * bw + 3, x0 + b * bw - 3
        o.append(f'<path d="M{xa} 92v-8H{xb}v8" fill="none" stroke="{col}" stroke-width="1.5"/>')
        o.append(f'<text x="{(xa + xb) / 2}" y="78" fill="{col}" font-size="13" text-anchor="middle">{name}</text>')
    # waveform from the CCR slots the encoder wrote
    path = f"M{x0 - 20} {ylo}H{x0}"
    for i, c in enumerate(d["ccr"][:16]):
        xs = x0 + i * bw
        xh = xs + bw * c / per
        path += f"V{yhi}H{xh:.1f}V{ylo}H{xs + bw}"
        o.append(f'<rect x="{xs}" y="{yhi - 6}" width="{bw}" height="{ylo - yhi + 12}" fill="none" '
                 f'stroke="{GRIDC}"/>')
        o.append(f'<text x="{xs + bw / 2}" y="{ylo + 26}" fill="{FG}" font-size="15" text-anchor="middle">'
                 f'{bits[i]}</text>')
        o.append(f'<text x="{xs + bw / 2}" y="{ylo + 46}" fill="{MUTED}" font-size="11" text-anchor="middle">'
                 f'CCR {c}</text>')
    gap_x = x0 + 16 * bw
    path += f"H{gap_x + 2 * bw * 0.5}"
    o.append(f'<path d="{path}" fill="none" stroke="{ACCENT}" stroke-width="2.5" stroke-linejoin="round"/>')
    o.append(f'<text x="{gap_x + 6}" y="{yhi + 40}" fill="{MUTED}" font-size="11">2 zero</text>')
    o.append(f'<text x="{gap_x + 6}" y="{yhi + 54}" fill="{MUTED}" font-size="11">slots</text>')
    o.append(f'<text x="{x0}" y="{H - 24}" fill="{MUTED}" font-size="12">Drawn from dshot_value_from_motor(), '
             'dshot_encode(), dshot_timing() and dshot_fill_ccr() in src/core/dshot.c '
             '(docs/assets/dshot_dump.c). Not a scope capture.</text>')
    o.append("</svg>")
    return "\n".join(o) + "\n"


# ------------------------------------------------------------------ terminal capture
def sim_main_capture():
    exe = os.path.join(ROOT, "build", "host", "sim_main" + (".exe" if os.name == "nt" else ""))
    runs = [["step"], ["gust"], ["gust", "ahrs_type=1"], ["failsafe"]]
    out = []
    for args in runs:
        r = subprocess.run([exe] + args, check=True, capture_output=True, text=True).stdout.strip()
        out.append(("./build/host/sim_main " + " ".join(args), r.splitlines()))
    return out


def terminal_svg(blocks, title) -> str:
    lh = 22
    n = sum(1 + len(b[1]) for b in blocks) + 1
    w, h = 980, 60 + lh * n
    o = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{w}" height="{h}" viewBox="0 0 {w} {h}">',
         f'<rect width="{w}" height="{h}" rx="10" fill="{BG}" stroke="{EDGE}"/>',
         f'<rect width="{w}" height="34" rx="10" fill="{GRIDC}"/><rect y="24" width="{w}" height="10" fill="{GRIDC}"/>',
         '<circle cx="20" cy="17" r="6" fill="#ff5f56"/><circle cx="40" cy="17" r="6" fill="#ffbd2e"/>'
         '<circle cx="60" cy="17" r="6" fill="#27c93f"/>',
         f'<text x="{w / 2}" y="22" fill="{MUTED}" font-family="{MONO}" font-size="13" text-anchor="middle">'
         f'{escape(title)}</text>', f'<g font-family="{MONO}" font-size="15">']
    y = 60
    for cmd, lines in blocks:
        o.append(f'<text x="20" y="{y}" fill="{FG}"><tspan fill="{ACCENT}">$</tspan> {escape(cmd)}</text>')
        y += lh
        for ln in lines:
            o.append(f'<text x="20" y="{y}" fill="{MUTED}">{escape(ln)}</text>')
            y += lh
    o.append(f'<text x="20" y="{y}" fill="{ACCENT}">$ <tspan fill="{FG}">_</tspan></text>')
    o += ["</g>", "</svg>"]
    return "\n".join(o) + "\n"


# ------------------------------------------------------------------ banner
def banner(step, d) -> str:
    W, H = 1280, 400
    o = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}">',
         "<defs>",
         f'<pattern id="g" width="32" height="32" patternUnits="userSpaceOnUse">'
         f'<path d="M32 0H0V32" fill="none" stroke="{GRIDC}" stroke-width="1"/></pattern>',
         f'<linearGradient id="fade" x1="0" x2="1"><stop offset="0" stop-color="{ACCENT}" stop-opacity="0"/>'
         f'<stop offset="0.25" stop-color="{ACCENT}" stop-opacity="1"/></linearGradient>',
         "</defs>",
         f'<rect width="{W}" height="{H}" fill="{BG}"/><rect width="{W}" height="{H}" fill="url(#g)"/>',
         f'<text x="72" y="186" font-family="{MONO}" font-weight="bold" font-size="96" fill="{FG}">skyforge-fc</text>',
         f'<text x="76" y="238" font-family="{MONO}" font-size="22" fill="{MUTED}">'
         'bare-metal STM32F405 flight controller</text>',
         f'<text x="76" y="270" font-family="{MONO}" font-size="22" fill="{MUTED}">'
         'for 5" FPV quads, written from scratch</text>',
         f'<rect x="76" y="300" width="420" height="4" rx="2" fill="{ACCENT}"/>',
         f'<text x="76" y="340" font-family="{MONO}" font-size="16" fill="{MUTED}">'
         'Madgwick + EKF · cascaded PID · DShot600 · CRSF/ELRS</text>']
    # motif 1: simulated roll-rate step response
    t0, sp, t90, ovs = step_metrics(step)
    w = [r for r in step if t0 - 0.05 <= r["t"] <= t0 + 0.4]
    X0, X1, Y0, Y1 = 790, 1220, 250, 70  # y: 0 dps at Y0, peak at Y1
    peak = max(r["p"] for r in w)
    sx = (X1 - X0) / (w[-1]["t"] - w[0]["t"])
    sy = (Y0 - Y1) / peak
    pts = " ".join(f"{X0 + (r['t'] - w[0]['t']) * sx:.1f},{Y0 - r['p'] * sy:.1f}" for r in w[::2])
    spp = " ".join(f"{X0 + (r['t'] - w[0]['t']) * sx:.1f},{Y0 - r['sp_p'] * sy:.1f}" for r in w[::2])
    o.append(f'<line x1="{X0}" y1="{Y0}" x2="{X1}" y2="{Y0}" stroke="{EDGE}"/>')
    o.append(f'<polyline points="{spp}" fill="none" stroke="{MUTED}" stroke-width="1.5" stroke-dasharray="5 5"/>')
    o.append(f'<polyline points="{pts}" fill="none" stroke="url(#fade)" stroke-width="3.5" '
             'stroke-linejoin="round" stroke-linecap="round"/>')
    o.append(f'<text x="{X1}" y="{Y1 - 12}" font-family="{MONO}" font-size="13" fill="{MUTED}" '
             f'text-anchor="end">sim: roll-rate step to {sp:.0f}°/s, t90 {t90 * 1000:.0f} ms</text>')
    # motif 2: the DShot600 frame from the real encoder
    bx0, bw, hi, lo = 790, (1220 - 790) / 16, 290, 330
    per = d["period"]
    path = f"M{bx0} {lo}"
    for i, c in enumerate(d["ccr"][:16]):
        xs = bx0 + i * bw
        path += f"V{hi}H{xs + bw * c / per:.1f}V{lo}H{xs + bw:.1f}"
    o.append(f'<path d="{path}" fill="none" stroke="{ACCENT}" stroke-width="2.5" stroke-linejoin="round"/>')
    o.append(f'<text x="1220" y="358" font-family="{MONO}" font-size="13" fill="{MUTED}" text-anchor="end">'
             f'DShot600 packet {d["packet"]} (throttle {d["value"]})</text>')
    o.append("</svg>")
    return "\n".join(o) + "\n"


def main() -> None:
    step = run_step()
    gm, ge = run_gust(0), run_gust(1)
    t90, ovs, pm, pe = hero_png(step, gm, ge, os.path.join(HERE, "sim_hero.png"))
    print(f"step: t90 {t90 * 1000:.1f} ms overshoot {ovs:.1f} %; gust peak roll Madgwick {pm:.1f} EKF {pe:.1f} deg")
    d = dshot_dump()
    write("dshot600.svg", dshot_svg(d))
    write("banner.svg", banner(step, d))
    write("sim_terminal.svg", terminal_svg(sim_main_capture(), "skyforge-fc simulator (sim_main)"))


if __name__ == "__main__":
    main()

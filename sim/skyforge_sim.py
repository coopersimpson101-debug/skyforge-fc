"""
skyforge_sim.py - ctypes bindings for the closed-loop simulator.

The shared library contains the *real* flight-controller C code
(src/core) plus the rigid-body model (sim/quad_sim.c). Build it with

    python tools/build.py sim

Example:

    from skyforge_sim import Sim
    s = Sim()
    s.set_param("p_roll", 60)
    s.arm(angle_mode=True)
    s.rc(throttle=s.hover_throttle())
    log = s.run(1.0)            # list of state dicts at 1 kHz
"""
import ctypes
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_EXT = ".dll" if sys.platform.startswith("win") else (".dylib" if sys.platform == "darwin" else ".so")
DEFAULT_LIB = os.path.join(ROOT, "build", "host", "skyforge_sim" + _EXT)

STATE_FIELDS = ["t", "roll", "pitch", "yaw", "p", "q", "r", "est_roll", "est_pitch", "est_yaw",
                "sp_p", "sp_q", "sp_r", "m1", "m2", "m3", "m4", "alt", "armed", "fs"]


class Sim:
    def __init__(self, lib_path=DEFAULT_LIB, seed=1):
        if not os.path.exists(lib_path):
            raise FileNotFoundError(f"{lib_path} not found - run `python tools/build.py sim`")
        self.lib = ctypes.CDLL(lib_path)
        L = self.lib
        L.sim_create.restype = ctypes.c_void_p
        L.sim_destroy.argtypes = [ctypes.c_void_p]
        L.sim_reset.argtypes = [ctypes.c_void_p, ctypes.c_uint32]
        L.sim_set_param.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_float]
        L.sim_set_param.restype = ctypes.c_int
        L.sim_set_rc.argtypes = [ctypes.c_void_p] + [ctypes.c_float] * 4 + [ctypes.c_int] * 2
        L.sim_set_link.argtypes = [ctypes.c_void_p, ctypes.c_int]
        L.sim_set_disturbance.argtypes = [ctypes.c_void_p] + [ctypes.c_float] * 3
        L.sim_kick_rates.argtypes = [ctypes.c_void_p] + [ctypes.c_float] * 3
        L.sim_step.argtypes = [ctypes.c_void_p, ctypes.c_int]
        L.sim_run_seconds.argtypes = [ctypes.c_void_p, ctypes.c_float]
        L.sim_arm.argtypes = [ctypes.c_void_p]
        L.sim_arm.restype = ctypes.c_int
        L.sim_get_state.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_float)]
        L.sim_hover_throttle.argtypes = [ctypes.c_void_p]
        L.sim_hover_throttle.restype = ctypes.c_float
        self.h = L.sim_create()
        L.sim_reset(self.h, seed)
        self._buf = (ctypes.c_float * len(STATE_FIELDS))()
        self._rc = dict(roll=0.0, pitch=0.0, yaw=0.0, throttle=0.0, arm=False, angle_mode=False)

    def __del__(self):
        if getattr(self, "h", None):
            self.lib.sim_destroy(self.h)
            self.h = None

    def set_param(self, name, value):
        if not self.lib.sim_set_param(self.h, name.encode(), float(value)):
            raise ValueError(f"unknown or out-of-range parameter {name}={value}")

    def rc(self, **kw):
        self._rc.update(kw)
        r = self._rc
        self.lib.sim_set_rc(self.h, r["roll"], r["pitch"], r["yaw"], r["throttle"],
                            int(r["arm"]), int(r["angle_mode"]))

    def link(self, up):
        self.lib.sim_set_link(self.h, int(up))

    def disturbance(self, tx=0.0, ty=0.0, tz=0.0):
        self.lib.sim_set_disturbance(self.h, tx, ty, tz)

    def arm(self, angle_mode=False):
        self._rc.update(angle_mode=angle_mode, throttle=0.0, arm=False)
        self.rc()
        ok = bool(self.lib.sim_arm(self.h))
        self._rc["arm"] = ok
        return ok

    def hover_throttle(self):
        return float(self.lib.sim_hover_throttle(self.h))

    def state(self):
        self.lib.sim_get_state(self.h, self._buf)
        return dict(zip(STATE_FIELDS, list(self._buf)))

    def run(self, seconds, log_hz=1000):
        """Advance the simulation, returning states sampled at log_hz."""
        out = []
        steps = int(round(seconds * log_hz))
        for _ in range(steps):
            self.lib.sim_run_seconds(self.h, 1.0 / log_hz)
            out.append(self.state())
        return out

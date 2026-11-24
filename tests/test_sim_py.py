"""
pytest: drive the C flight controller through ctypes in the simulator.
Requires the shared library (python tools/build.py sim).
"""
import os
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "sim"))

from skyforge_sim import DEFAULT_LIB, Sim  # noqa: E402

pytestmark = pytest.mark.skipif(not os.path.exists(DEFAULT_LIB),
                                reason="build the simulator first: python tools/build.py sim")


def hover(s, angle):
    assert s.arm(angle_mode=angle)
    hov = s.hover_throttle()
    s.rc(throttle=hov + 0.08)
    s.run(1.0, log_hz=100)
    s.rc(throttle=hov)
    s.run(1.0, log_hz=100)
    assert s.state()["alt"] > 0.3


@pytest.mark.parametrize("ahrs", [0, 1])
def test_stabilises_from_disturbed_attitude(ahrs):
    s = Sim()
    s.set_param("ahrs_type", ahrs)
    hover(s, angle=True)
    s.disturbance(0.6, -0.25, 0.0)
    log = s.run(0.15)
    s.disturbance(0, 0, 0)
    log += s.run(1.0)
    peak = max(abs(r["roll"]) for r in log)
    final = log[-1]
    assert peak > 15.0
    assert abs(final["roll"]) < 3.0 and abs(final["pitch"]) < 3.0
    assert final["armed"] == 1.0


def test_param_validation():
    s = Sim()
    with pytest.raises(ValueError):
        s.set_param("p_roll", 1e6)
    with pytest.raises(ValueError):
        s.set_param("does_not_exist", 1)


def test_disarmed_motors_stay_off():
    s = Sim()
    s.rc(throttle=0.0, arm=False)
    log = s.run(2.0, log_hz=50)
    assert all(r["armed"] == 0.0 and r["m1"] == 0.0 for r in log)
    assert log[-1]["alt"] == 0.0

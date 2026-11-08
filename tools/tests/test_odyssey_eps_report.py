"""The characterization report recovers known steering behaviour from runs buried in lane-keeping noise."""
import numpy as np

from tools.odyssey_eps_report import STEER_MAX, fine_angle, held_pulses, pulse_report, schedule
from openpilot.common.test import OpenpilotTestCase


def build(names: list[str], respond, runs_each: int = 4, seed: int = 0):
  """A drive made of runs of the named maneuvers, separated by gaps; respond(request, name) -> dict of signal deltas."""
  rng = np.random.default_rng(seed)
  cols = {k: [] for k in ("t", "v", "fine", "vsa_lat", "cmd_tq", "motor_tq", "maneuver")}
  by_name, i = {}, 0
  for idx, name in enumerate(names):
    f, m = schedule(name)
    n = int(m.actions[0].time_bp[-1] * 100) + 200
    req = f(np.arange(n) / 100)
    for _ in range(runs_each):
      sig = respond(req, name, rng)
      for k in ("fine", "vsa_lat", "cmd_tq", "motor_tq"):
        cols[k].append(sig.get(k, np.zeros(n)))
      cols["maneuver"].append(np.full(n, idx))
      cols["v"].append(np.full(n, 25.0))
      by_name.setdefault(name, []).append(np.arange(i, i + n))
      i += n
      for k in ("fine", "vsa_lat", "cmd_tq", "motor_tq", "v"):  # a gap between runs
        cols[k].append(np.zeros(100))
      cols["maneuver"].append(np.full(100, -1))
      i += 100
  d = {k: np.concatenate(v) for k, v in cols.items() if v}
  d["t"] = np.arange(len(d["maneuver"])) / 100
  return d, by_name


class TestOdysseyEpsReport(OpenpilotTestCase):
  def test_fine_angle_sees_motion_between_ticks(self):
    rng = np.random.default_rng(0)
    t = np.arange(3000) / 100
    true = 0.15 * np.sin(2 * np.pi * t / 10)  # slow +-0.15 degree wheel motion
    rate = np.round((np.gradient(true) * 100 + rng.normal(0, 0.05, len(t))) / 0.1) * 0.1  # 0.1 deg/s steps, noisy
    reading = np.round(true / 0.1) * 0.1
    fine = fine_angle(reading, rate)
    assert np.std(fine - true) < 0.5 * np.std(reading - true)

  def test_held_pulses_find_the_smallest_pulse_that_moves_the_wheel(self):
    threshold = 0.05

    def respond(req, name, rng):
      wheel = np.cumsum(np.where(np.abs(req) > threshold, np.sign(req) * 0.004, 0.0))  # turns only past the deadband
      wheel += np.cumsum(rng.normal(0, 0.001, len(req)))
      return {"fine": wheel, "cmd_tq": -req * STEER_MAX, "motor_tq": np.abs(req) * 90, "vsa_lat": 0.05 * wheel}
    d, by_name = build(["held pulses 0.5s left first", "held pulses 0.5s right first"], respond, runs_each=2)
    pulses = held_pulses(d, by_name)
    assert len(pulses) == 2 * 2 * 10
    assert {p["level"] for p in pulses} == {0.0, 0.03, 0.045, 0.06, 0.08}
    assert pulse_report(pulses) == 0.06

  def test_wander_ab_split_follows_the_kick_blocks(self):
    from openpilot.sunnypilot.selfdrive.controls.lib.steer_deadband import AB_BLOCK_S, ab_kick_on
    from tools.odyssey_wander import FS, ab_split
    t = np.arange(0, 4 * AB_BLOCK_S, 1 / FS) + 10.0
    d = {"t": t}
    pieces = ab_split(d, [np.arange(len(t))], 60)
    assert [on for on, _ in pieces] == [False, True, False, True]  # the last 10 s piece is too short to score
    for on, p in pieces:
      assert all(ab_kick_on(x) == on for x in t[p])

"""The Odyssey characterization maneuvers stay small, balanced, and hold lane keeping still around each pulse."""
import numpy as np

from openpilot.common.constants import CV
from openpilot.common.parameterized import parameterized
from openpilot.common.realtime import DT_MDL
from openpilot.common.test import OpenpilotTestCase
from openpilot.tools.lateral_maneuvers.odyssey_suite import maneuvers

N = len(maneuvers())


def run_once(m) -> np.ndarray:
  """Step one run of a maneuver the way lateral_maneuversd does, after it has become active."""
  m._active = True
  out = []
  while True:
    out.append(m._step())
    if m._run_completed:
      return np.array(out)


def frames_to_start(m, speeds_mph) -> int | None:
  for i, mph in enumerate(speeds_mph):
    m.get_accel(mph * CV.MPH_TO_MS, True, 0.0, 0.0)
    if m.active:
      return i
  return None


class TestOdysseySuite(OpenpilotTestCase):
  def test_both_directions(self):
    names = [m.description for m in maneuvers()]
    assert any("left first" in n for n in names) and any("right first" in n for n in names)
    assert len(set(names)) == len(names), "names key the progress saved across drives"
    assert all(m.follow_lane and m.torque for m in maneuvers()), "torque pulses riding on lane keeping"

  @parameterized.expand([40, 47, 58, 72])
  def test_starts_at_any_steady_highway_speed(self, mph):
    i = frames_to_start(maneuvers()[0], [mph] * 100)
    assert i is not None and i * DT_MDL < 2.5

  def test_waits_while_speed_changes_or_too_slow(self):
    assert frames_to_start(maneuvers()[0], np.linspace(50, 70, 200)) is None  # accelerating ~1 mph/s
    assert frames_to_start(maneuvers()[0], [35] * 200) is None

  @parameterized.expand(range(N))
  def test_torque_pulses_small_balanced_and_sharp(self, idx):
    tq = run_once(maneuvers()[idx])
    assert np.abs(tq).max() <= 0.08 + 1e-9, "stays well below what the driver would notice as a tug"
    assert abs(tq.sum() * DT_MDL) < 1e-6, "net torque zero, so the van isn't pushed off its line"
    assert tq[-1] == 0.0
    vals, counts = np.unique(np.round(np.abs(tq), 4), return_counts=True)
    levels = vals[(vals > 0) & (counts >= 4)]  # pulse plateaus, not the one-frame edges
    assert len(levels) == 4 and levels.min() < 0.037 < levels.max(), "brackets the measured deadband"

  @parameterized.expand(range(N))
  def test_torque_pulses_last_their_width(self, idx):
    m = maneuvers()[idx]
    on = np.abs(run_once(m)) > 0
    edges = np.flatnonzero(np.diff(np.r_[0, on.astype(int), 0]))
    widths = (edges[1::2] - edges[::2]) * DT_MDL
    assert np.allclose(widths, float(m.description.split()[2].rstrip("s")))

  @parameterized.expand(range(N))
  def test_each_pulse_sits_inside_its_own_hold(self, idx):
    m = maneuvers()[idx]
    tq = run_once(m)
    t = np.arange(len(tq)) * DT_MDL
    holds = np.array(m.holds)
    assert len(holds) == 10 and np.all(holds[1:, 0] > holds[:-1, 1]), "one hold per pulse, with lane keeping between"
    for on in t[np.abs(tq) > 0]:
      a, b = holds[(holds[:, 0] <= on) & (on < holds[:, 1])][0]
      assert on - a >= 0.25 and b - on >= 0.65, "held before and after the pulse"

  def test_hold_freezes_lane_keeping_torque(self):
    m = maneuvers()[0]
    m._active = True
    rng = np.random.default_rng(0)
    steer, cmds, pulses, held = 0.0, [], [], []
    while True:
      pulse = m._step()
      steer += rng.normal(0, 0.005)  # lane keeping's torque wanders
      cmds.append(steer + pulse + m.hold_correction(steer))
      pulses.append(pulse)
      held.append(m._held is not None)
      if m._run_completed:
        break
    cmds, pulses, held = np.array(cmds), np.array(pulses), np.array(held)
    edges = np.flatnonzero(np.diff(np.r_[0, held.astype(int), 0]))
    for a, b in zip(edges[::2], edges[1::2], strict=True):
      base = cmds[a:b] - pulses[a:b]
      assert np.ptp(base) < 1e-9, "the command moves only by the pulse while held"
    assert not held[-1] and m._held is None

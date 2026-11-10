"""Takeover report: presses are sorted by cause from the request, the command and the driver's torque direction."""
import unittest

import numpy as np

from tools.odyssey_takeovers import FS, low_speed, presses


def drive(seconds=60, v=15.0):
  n = seconds * FS
  d = {k: np.zeros(n) for k in ("seg", "pressed", "drv", "blinker", "angle", "torque", "des_curv", "cur_curv")}
  d.update(t=np.arange(n) / FS, v=np.full(n, v), lat=np.ones(n))
  return d


def press(d, start_s, dur_s, driver, want_accel, command):
  a, b = int(start_s * FS), int((start_s + dur_s) * FS)
  d["pressed"][a:b] = 1
  d["drv"][a:b] = driver
  d["des_curv"][a - 2 * FS:b] = want_accel / d["v"][a]**2
  d["torque"][a - 2 * FS:b] = command


class TestOdysseyTakeovers(unittest.TestCase):
  def test_presses_are_sorted_by_cause(self):
    d = drive()
    press(d, 5, 1.0, driver=-2000, want_accel=+2.0, command=-1.0)  # right curve, flat out, driver adds right turn
    press(d, 15, 1.0, driver=-2000, want_accel=+2.0, command=-0.4)  # the same with torque to spare
    press(d, 25, 1.0, driver=+2000, want_accel=+2.0, command=-0.4)  # driver steers against the turn
    press(d, 35, 1.0, driver=+1500, want_accel=0.1, command=0.05)  # straight road
    press(d, 45, 0.1, driver=+1500, want_accel=0.1, command=0.05)  # a sensor blip
    self.assertEqual([p["kind"] for p in presses(d)],
                     ["curve, at the limit", "curve, adding turn", "curve, easing turn", "straight"])

  def test_blinker_slow_and_inactive_presses_are_left_out(self):
    d = drive()
    press(d, 5, 1.0, -2000, 2.0, -1.0)
    d["blinker"][4 * FS:5 * FS] = 1
    press(d, 25, 1.0, -2000, 2.0, -1.0)
    d["lat"][20 * FS:30 * FS] = 0
    self.assertEqual(presses(d), [])
    slow = drive(v=5.0)
    press(slow, 5, 1.0, -2000, 0.5, -1.0)
    self.assertEqual(presses(slow), [])

  def test_low_speed_tells_a_soft_controller_from_a_spent_eps(self):
    d = drive(v=3.5)
    k = -900.0  # wheel degrees per unit curvature: the opposite sign, as on the van
    d["cur_curv"][:] = 0.01
    d["angle"][:] = k * 0.01
    d["des_curv"][:] = 0.02  # the request needs 9 degrees more wheel than the van has
    d["torque"][:30 * FS] = -0.5
    d["torque"][30 * FS:] = -1.0
    band, secs, share, free_s, cmd, full = low_speed(d)[0]
    self.assertEqual((band, secs, share, free_s), ("<10", 60.0, 1.0, 60.0))
    self.assertAlmostEqual(full, 0.5)
    arm = np.arange(len(d["t"])) < 30 * FS
    self.assertEqual(low_speed(d, arm)[0][5], 0.0)
    self.assertEqual(low_speed(d, ~arm)[0][5], 1.0)

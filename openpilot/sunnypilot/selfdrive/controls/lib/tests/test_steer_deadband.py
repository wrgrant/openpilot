import numpy as np

from openpilot.sunnypilot.selfdrive.controls.lib.steer_deadband import deadband_comp


def test_off_is_identity():
  for t in (-0.3, -0.01, 0.0, 0.02, 0.5):
    assert deadband_comp(t, 0.0) == t


def test_zero_stays_zero_and_sign_is_kept():
  assert deadband_comp(0.0, 0.037) == 0.0
  for t in (-0.2, -0.004, 0.004, 0.2):
    assert np.sign(deadband_comp(t, 0.037)) == np.sign(t)


def test_small_requests_clear_the_deadband():
  # any request beyond a couple of smoothing widths gets the full kick on top
  for t in (0.03, 0.05, -0.03, -0.05):
    assert abs(deadband_comp(t, 0.037)) >= abs(t) + 0.037 * 0.99


def test_monotonic():
  x = np.linspace(-0.3, 0.3, 2001)
  y = np.array([deadband_comp(t, 0.037) for t in x])
  assert np.all(np.diff(y) > 0)

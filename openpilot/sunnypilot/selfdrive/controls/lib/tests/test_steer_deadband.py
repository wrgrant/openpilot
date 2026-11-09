import numpy as np

from openpilot.sunnypilot.selfdrive.controls.lib.steer_deadband import deadband_comp
from openpilot.common.test import OpenpilotTestCase


class TestSteerDeadband(OpenpilotTestCase):
  def test_bump_averages_to_the_request(self):
    from openpilot.sunnypilot.selfdrive.controls.lib.steer_deadband import BUMP_PERIOD_FRAMES, bump_comp
    for t in (0.012, -0.024, 0.04):
      out = [bump_comp(t, 0.06, k) for k in range(BUMP_PERIOD_FRAMES)]
      assert abs(np.mean(out) - t) <= 0.06 / BUMP_PERIOD_FRAMES / 2 + 1e-9
      assert set(np.round(np.abs(out), 6)) <= {0.0, 0.06}, "pulses are full size or nothing"

  def test_bump_passes_large_requests_and_ignores_noise(self):
    from openpilot.sunnypilot.selfdrive.controls.lib.steer_deadband import bump_comp
    assert bump_comp(0.2, 0.06, 0) == 0.2 and bump_comp(-0.07, 0.06, 3) == -0.07
    assert bump_comp(0.001, 0.06, 0) == 0.0

  def test_off_is_identity(self):
    for t in (-0.3, -0.01, 0.0, 0.02, 0.5):
      assert deadband_comp(t, 0.0) == t

  def test_zero_stays_zero_and_sign_is_kept(self):
    assert deadband_comp(0.0, 0.037) == 0.0
    for t in (-0.2, -0.004, 0.004, 0.2):
      assert np.sign(deadband_comp(t, 0.037)) == np.sign(t)

  def test_small_requests_clear_the_deadband(self):
    # any request beyond a couple of smoothing widths gets the full kick on top
    for t in (0.03, 0.05, -0.03, -0.05):
      assert abs(deadband_comp(t, 0.037)) >= abs(t) + 0.037 * 0.99

  def test_monotonic(self):
    x = np.linspace(-0.3, 0.3, 2001)
    y = np.array([deadband_comp(t, 0.037) for t in x])
    assert np.all(np.diff(y) > 0)

  def test_ab_mode_rotates_off_kick_bump(self):
    from openpilot.sunnypilot.selfdrive.controls.lib.odyssey_ab import AB_BLOCK_S
    from openpilot.sunnypilot.selfdrive.controls.lib.steer_deadband import AB_KICK, SteerDeadbandComp

    class FakeParams:
      def get(self, key, return_default=False):
        return 0.0

      def get_bool(self, key):
        return key == "SteerDeadbandCompAB"

    comp = SteerDeadbandComp()
    comp._params = FakeParams()
    off = comp.update(0.05, 1.0, mono_s=0.5 * AB_BLOCK_S)
    kick = comp.update(0.05, 1.0, mono_s=1.5 * AB_BLOCK_S)
    bump = [comp.update(0.02, 1.0, mono_s=2.5 * AB_BLOCK_S) for _ in range(25)]
    assert off == 0.05
    assert abs(kick - (0.05 + AB_KICK * np.tanh(5))) < 1e-9
    assert set(np.round(bump, 6)) == {0.0, 0.06}

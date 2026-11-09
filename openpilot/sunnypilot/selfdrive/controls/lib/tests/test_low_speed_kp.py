"""The low-speed gain cap only lowers the walking-speed gains, and off leaves openpilot's gains alone."""
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.controls.lib.latcontrol_torque import KP_INTERP
from openpilot.sunnypilot.selfdrive.controls.lib.low_speed_angle import capped_kp


class TestLowSpeedKpCap(OpenpilotTestCase):
  def test_off_is_unchanged(self):
    assert capped_kp(KP_INTERP, 0.0) == KP_INTERP

  def test_caps_low_speed_only(self):
    out = capped_kp(KP_INTERP, 10.0)
    assert max(out) == 10.0
    assert out[-1] == KP_INTERP[-1], "highway gain untouched"
    assert all(o <= k for o, k in zip(out, KP_INTERP, strict=True))

  def test_never_below_highway_gain(self):
    assert capped_kp(KP_INTERP, 0.1) == [max(min(k, KP_INTERP[-1]), KP_INTERP[-1]) for k in KP_INTERP]

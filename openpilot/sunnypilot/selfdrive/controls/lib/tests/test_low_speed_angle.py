"""Low-speed angle steering: blends by speed, pushes toward the wanted wheel angle, stays bounded, off is a no-op."""
from types import SimpleNamespace as NS

import numpy as np

from openpilot.common.test import OpenpilotTestCase
from openpilot.sunnypilot.selfdrive.controls.lib import low_speed_angle as lsa


class FakeVM:
  def get_steer_from_curvature(self, curv, u, roll):
    return curv * 3.0 * 17.0  # small-angle kinematic steering wheel angle, radians


class TestLowSpeedAngle(OpenpilotTestCase):
  def test_blend_weight(self):
    assert lsa.blend_weight(1.0) == 1.0 and lsa.blend_weight(lsa.V_OFF + 1) == 0.0
    assert 0.0 < lsa.blend_weight((lsa.V_FULL + lsa.V_OFF) / 2) < 1.0

  def test_pushes_toward_the_wanted_angle_and_is_bounded(self):
    assert lsa.angle_steer(0.0, -10.0) > 0 and lsa.angle_steer(0.0, 10.0) < 0
    assert lsa.angle_steer(30.0, 30.0) == lsa.HOLD_MAX, "holds a wanted angle with the hold term only"
    for des, ang in ((400.0, -400.0), (-400.0, 400.0), (5.0, 3.0)):
      assert -1.0 <= lsa.angle_steer(des, ang) <= 1.0

  def test_small_errors_ask_for_little_torque(self):
    # the case that saturated before: near straight, a few degrees off
    assert abs(lsa.angle_steer(2.7, 8.7)) < 0.15

  def test_off_and_high_speed_leave_the_stock_command(self):
    vm, live = FakeVM(), NS(roll=0.0, angleOffsetDeg=0.0)
    off = lsa.LowSpeedAngleSteer(params=NS(get_bool=lambda k: False))
    assert off.blend(0.3, 2.0, 0.0, vm, live, 0.01) == 0.3
    on = lsa.LowSpeedAngleSteer(params=NS(get_bool=lambda k: True))
    assert on.blend(0.3, lsa.V_OFF + 1, 0.0, vm, live, 0.01) == 0.3

  def test_gain_ab_alternates_by_minute_and_needs_angle_steering_on(self):
    vm, live = FakeVM(), NS(roll=0.0, angleOffsetDeg=0.0)
    ab = lsa.LowSpeedAngleSteer(params=NS(get_bool=lambda k: True))
    out = [ab.blend(0.0, 1.0, 5.0, vm, live, 0.0, mono_s=60.0 * block + 1) for block in range(4)]
    assert out[0] == out[2] == -lsa.K_ERR * 5 and out[1] == out[3] == -lsa.K_ERR_FIRM * 5
    plain = lsa.LowSpeedAngleSteer(params=NS(get_bool=lambda k: k == "SteerLowSpeedAngle"))
    assert plain.blend(0.0, 1.0, 5.0, vm, live, 0.0, mono_s=61.0) == -lsa.K_ERR * 5
    ab_only = lsa.LowSpeedAngleSteer(params=NS(get_bool=lambda k: k == "SteerLowSpeedAngleAB"))
    assert ab_only.blend(0.3, 1.0, 5.0, vm, live, 0.0, mono_s=61.0) == 0.3

  def test_sign_matches_openpilot(self):
    # a left curve (+ curvature) wants a negative steering angle here (latcontrol_angle's convention); from a straight
    # wheel the command (actuators.torque, same sign as the steering angle) moves toward it, so it is negative
    vm, live = FakeVM(), NS(roll=0.0, angleOffsetDeg=0.0)
    on = lsa.LowSpeedAngleSteer(params=NS(get_bool=lambda k: True))
    out = on.blend(0.0, 1.0, 0.0, vm, live, +0.01)
    assert out < 0 and np.isfinite(out)

  def test_kp_cap_follows_the_setting(self):
    kp = [250.0, 30.0, 1.0]
    on = NS(get_bool=lambda k: k == "SteerLowSpeedAngle", get=lambda k, return_default=False: 0.0)
    off = NS(get_bool=lambda k: False, get=lambda k, return_default=False: 0.0)
    assert lsa.odyssey_kp_interp(kp, on) == [lsa.STOCK_KP_CAP, lsa.STOCK_KP_CAP, 1.0]
    assert lsa.odyssey_kp_interp(kp, off) == kp

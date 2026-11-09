"""Odyssey: steer on wheel angle instead of lateral accel at low speed (SteerLowSpeedAngle, off by default).

At walking speed a real curve needs almost no lateral accel, so openpilot's torque controller raises its gain up to
300x (KP_INTERP) to steer at all. Against the Odyssey's weak EPS that turns a few degrees of wheel error into 100%
torque back and forth. The wheel angle, though, is measured to 0.1 degree and the angle a curve needs is large, so below
V_FULL this steers on angle: a hold term for the wheel angle the curve needs, plus a correction on the angle error.
It blends back to the stock controller by V_OFF, and in between the stock gain is capped so the two don't fight.
"""
import math

import numpy as np

from openpilot.common.params import Params

V_FULL, V_OFF = 4.5, 8.0  # m/s (10 and 18 mph): all angle steering below V_FULL, all stock above V_OFF
K_HOLD = 0.02  # torque (fraction of max) per degree of wanted wheel angle, to hold the wheel there
HOLD_MAX = 0.35  # holding torque measured on routes 1a7/1aa/1ab tops out around 0.4
K_ERR = 0.02  # torque per degree of wheel angle error: 50 degrees short asks for full torque
STOCK_KP_CAP = 8.0  # stock proportional gain cap while this is on (openpilot's is 250 at 1 m/s)


def blend_weight(v_ego: float) -> float:
  return float(np.interp(v_ego, [V_FULL, V_OFF], [1.0, 0.0]))


def angle_steer(angle_des: float, angle: float) -> float:
  """Steering command (openpilot's actuators.torque sign, same as the steering angle's) to reach angle_des degrees."""
  hold = float(np.clip(K_HOLD * angle_des, -HOLD_MAX, HOLD_MAX))
  return float(np.clip(hold + K_ERR * (angle_des - angle), -1.0, 1.0))


class LowSpeedAngleSteer:
  def __init__(self, params: Params | None = None):
    self.enabled = bool((params or Params()).get_bool("SteerLowSpeedAngle"))

  def blend(self, output_torque: float, v_ego: float, angle_deg: float, VM, live_params, desired_curvature: float) -> float:
    """output_torque is latcontrol_torque's internal sign (the negative of actuators.torque)."""
    if not self.enabled:
      return output_torque
    w = blend_weight(v_ego)
    if w <= 0.0:
      return output_torque
    angle_des = math.degrees(VM.get_steer_from_curvature(-desired_curvature, v_ego, live_params.roll)) + live_params.angleOffsetDeg
    return (1.0 - w) * output_torque + w * -angle_steer(angle_des, angle_deg)

"""Odyssey: steer on wheel angle instead of lateral accel at low speed (SteerLowSpeedAngle, off by default).

At walking speed a real curve needs almost no lateral accel, so openpilot's torque controller raises its gain up to
300x (KP_INTERP) to steer at all. Against the Odyssey's weak EPS that turns a few degrees of wheel error into 100%
torque back and forth. The wheel angle, though, is measured to 0.1 degree and the angle a curve needs is large, so below
V_FULL this steers on angle: a hold term for the wheel angle the curve needs, plus a correction on the angle error.
It blends back to the stock controller by V_OFF, and in between the stock gain is capped so the two don't fight.
Applied by odyssey_steer.OdysseySteer after whichever torque controller sunnypilot runs (v0 or v1).
"""
import math
import time

import numpy as np

from openpilot.common.params import Params
from openpilot.sunnypilot.selfdrive.controls.lib.odyssey_ab import LOWSPEED_ARMS, ab_arm

V_FULL, V_OFF = 4.5, 8.0  # m/s (10 and 18 mph): all angle steering below V_FULL, all stock above V_OFF
K_HOLD = 0.02  # torque (fraction of max) per degree of wanted wheel angle, to hold the wheel there
HOLD_MAX = 0.35  # holding torque measured on routes 1a7/1aa/1ab tops out around 0.4
K_ERR = 0.02  # torque per degree of wheel angle error: 50 degrees short asks for full torque
# SteerLowSpeedAngleAB "firm" arm: 12 degrees short asks for full torque. On the first drives with K_ERR (1ac-1ae) the
# wheel sat a median 11 degrees short, hands off, at half command, and below 10 mph it only moves (about 8 deg/s) near
# full command. At 8-18 deg/s per unit command and a 0.5 s steering delay, 0.08 keeps a 50-70 degree phase margin.
K_ERR_FIRM = 0.08
STOCK_KP_CAP = 8.0  # stock proportional gain cap while this is on (openpilot's is 250 at 1 m/s)


def blend_weight(v_ego: float) -> float:
  return float(np.interp(v_ego, [V_FULL, V_OFF], [1.0, 0.0]))


def angle_steer(angle_des: float, angle: float, k_err: float = K_ERR) -> float:
  """Steering command (openpilot's actuators.torque sign, same as the steering angle's) to reach angle_des degrees."""
  hold = float(np.clip(K_HOLD * angle_des, -HOLD_MAX, HOLD_MAX))
  return float(np.clip(hold + k_err * (angle_des - angle), -1.0, 1.0))


class LowSpeedAngleSteer:
  def __init__(self, params: Params | None = None):
    p = params or Params()
    self.enabled = bool(p.get_bool("SteerLowSpeedAngle"))
    self.ab = self.enabled and bool(p.get_bool("SteerLowSpeedAngleAB"))

  def k_err(self, mono_s: float | None = None) -> float:
    if not self.ab:
      return K_ERR
    firm = ab_arm(time.monotonic() if mono_s is None else mono_s, LOWSPEED_ARMS) == "firm"
    return K_ERR_FIRM if firm else K_ERR

  def blend(self, steer: float, v_ego: float, angle_deg: float, VM, live_params, desired_curvature: float,
            mono_s: float | None = None) -> float:
    """steer is the lateral controller's command in actuators.torque sign (same sign as the steering angle)."""
    if not self.enabled:
      return steer
    w = blend_weight(v_ego)
    if w <= 0.0:
      return steer
    angle_des = math.degrees(VM.get_steer_from_curvature(-desired_curvature, v_ego, live_params.roll)) + live_params.angleOffsetDeg
    return (1.0 - w) * steer + w * angle_steer(angle_des, angle_deg, self.k_err(mono_s))


def capped_kp(kp_interp: list[float], cap: float) -> list[float]:
  """openpilot raises the torque controller's proportional gain up to 300x at walking speed; cap it (0 = off), never
  below the highway gain. Used while low-speed angle steering is on, so the stock controller doesn't fight it in the
  blend region."""
  if cap <= 0.0:
    return kp_interp
  return [min(k, max(cap, kp_interp[-1])) for k in kp_interp]


def odyssey_kp_interp(kp_interp: list[float], params: Params | None = None) -> list[float]:
  """The torque controllers' KP table with the Odyssey low-speed cap: STOCK_KP_CAP while low-speed angle steering is
  on, else SteerLowSpeedKpCap (0 = off)."""
  p = params or Params()
  cap = STOCK_KP_CAP if p.get_bool("SteerLowSpeedAngle") else float(p.get("SteerLowSpeedKpCap", return_default=True) or 0.0)
  return capped_kp(kp_interp, cap)

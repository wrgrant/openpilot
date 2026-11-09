"""Experimental curve preview/return primitives. NumPy only, default OFF.

Angles/rates use Honda wheel sign. Acceleration/command use OP curvature sign
(opposite Honda torqueState.output/actuators.torque). Caller owns plan timestamp,
VehicleModel angle conversion, PID anti-windup and final CAN safety/slew limits.
No online plant identification or claim of road validation.
"""

import numpy as np


class CurveTransitions:
  def __init__(self, *, enabled=False, preview=False, return_mode='off', horizon=1.0, predictor='sample', preview_gain=0.5, return_limit=0.12):
    if predictor not in ("sample", "inverse"):
      raise ValueError("predictor must be sample or inverse")
    self.predictor = predictor
    if return_mode not in ('off', 'release', 'active'):
      raise ValueError('return_mode must be off, release or active')
    if not np.isfinite([horizon, preview_gain, return_limit]).all() or not (0 < horizon <= 2 and 0 <= preview_gain <= 1 and 0 <= return_limit <= 0.2):
      raise ValueError('invalid configuration')
    self.enabled, self.preview, self.return_mode = enabled, preview, return_mode
    self.horizon, self.preview_gain, self.return_limit = horizon, preview_gain, return_limit
    self.reset()

  def reset(self):
    self.correction = self.return_delta = self.return_time = 0.0
    self.mode = 'off'

  def update(
    self,
    *,
    command,
    target_accel,
    speed,
    angle_deg,
    rate_deg_s,
    planned_angle_deg,
    plan_t,
    plan_accel,
    active,
    driver_pressed,
    driver_torque,
    plan_age,
    dt=0.01,
  ):
    """Return (adjusted target acceleration, adjusted normalized command).

    For runtime: preview is consumed BEFORE LaC; return command AFTER LaC.
    Use two instances (preview-only, return-only), once each/frame. Never feed
    output torque to another controller as a target. Missing/stale data bypasses
    exactly to stock and clears state; stock is responsible for its own bounds.
    """
    stock = (target_accel, command)
    vals = [command, target_accel, speed, angle_deg, rate_deg_s, planned_angle_deg, driver_torque, plan_age, dt]
    try:
      t, a = np.asarray(plan_t, dtype=float), np.asarray(plan_accel, dtype=float)
    except (TypeError, ValueError):
      self.reset()
      return stock
    valid = (
      self.enabled
      and active
      and not driver_pressed
      and np.isfinite(vals).all()
      and abs(driver_torque) < 400
      and 5 <= speed <= 40
      and 0 <= plan_age <= 0.15
      and 0.001 <= dt <= 0.05
      and abs(command) <= 1
      and t.ndim == a.ndim == 1
      and len(t) == len(a)
      and 3 <= len(t) <= 128
      and np.isfinite(t).all()
      and np.isfinite(a).all()
      and (np.diff(t) > 0).all()
      and t[0] <= 0
      and t[-1] >= self.horizon + plan_age
    )
    if not valid:
      self.reset()
      return stock
    # Difference anchoring preserves the action/current target's feedback and
    # avoids replacing it with the model trajectory's biased zero-time sample.
    now = float(np.interp(plan_age, t, a))
    future = float(np.interp(self.horizon + plan_age, t, a))
    middle = float(np.interp(self.horizon / 2 + plan_age, t, a))
    delta = future - now
    if self.predictor == "inverse":
      # First-order inverse (tau*desired rate), not a fictitious 1 s delay.
      delta = 1.061 * (float(np.interp(0.5 + plan_age, t, a)) - now) / 0.5
    coherent = (middle - now) * delta > 0 and abs(delta) > 0.05
    desired = float(np.clip(self.preview_gain * delta, -0.3, 0.3)) if self.preview and coherent else 0.0
    self.correction += (desired - self.correction) * dt / (0.25 + dt)
    target = target_accel + self.correction
    # Return only toward an explicitly straighter planned wheel angle, with
    # no opposite-curve plan and a stopping-distance guard on measured rate.
    sign = float(np.sign(angle_deg))
    gap = abs(angle_deg) - abs(planned_angle_deg)
    returning = (
      abs(angle_deg) > 2
      and gap > 1
      and abs(planned_angle_deg) < 0.6 * abs(angle_deg)
      and angle_deg * planned_angle_deg >= 0
      and sign * rate_deg_s < 1
      and abs(angle_deg + rate_deg_s * 0.4) > abs(planned_angle_deg) + 0.5
      and (angle_deg + rate_deg_s * 0.4) * angle_deg > 0
    )
    self.return_time = self.return_time + dt if returning else 0.0
    eligible = returning and self.return_time >= 0.15 and self.return_mode != 'off'
    request = 0.0
    self.mode = 'preview' if abs(self.correction) > 0.01 else 'off'
    if eligible:
      # Positive sign*command is torque toward straight (command sign is OP).
      velocity_goal = -sign * min(8.0, 1.5 * gap)
      if self.return_mode == 'release':
        wanted = command if sign * command >= 0 else 0.0
      else:
        needed = max(0.0, -sign * (velocity_goal - rate_deg_s)) * 0.025
        wanted = sign * min(self.return_limit, needed)
        # Never weaken an already stronger stock return command.
        wanted = sign * max(sign * command, sign * wanted)
      request = float(np.clip(wanted - command, -0.2, 0.2))
      self.mode = self.return_mode
    # Bound intervention slew independently; final CAN slew still applies.
    # Disarm immediately at stopping guard: no residual reverse pulse at centre.
    if not returning:
      self.return_delta = 0.0
    else:
      self.return_delta += float(np.clip(request - self.return_delta, -0.5 * dt, 0.5 * dt))
    return float(target), float(np.clip(command + self.return_delta, -1, 1))

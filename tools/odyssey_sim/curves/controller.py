"""Offline calib hook. Runtime integration contract is in README.md."""

import math
from tools.odyssey_sim.calib.controller import Controller as Base
from tools.odyssey_sim.curves.transition import CurveTransitions
from openpilot.selfdrive.modeld.constants import ModelConstants


class Controller(Base):
  def __init__(self, meta, cfg, preview=False, return_mode='off', horizon=1.0, preview_gain=0.5):
    super().__init__(meta, cfg)
    self.pre = CurveTransitions(enabled=True, preview=preview, horizon=horizon, preview_gain=preview_gain)
    self.ret = CurveTransitions(enabled=True, return_mode=return_mode)

  def update(self, target_lataccel, current_lataccel, state, future_plan=None):
    if future_plan is None:
      self.pre.reset()
      self.ret.reset()
      return super().update(target_lataccel, current_lataccel, state, future_plan)
    v = float(state.v_ego)
    self.vm.update_params(max(state.stiffness, 0.1), max(state.ratio, 0.1))

    def angle(accel):
      return math.degrees(self.vm.get_steer_from_curvature(-accel / max(v * v, 0.1), v, state.roll))

    wheel = angle(current_lataccel)
    # Prior-frame angle is available from the unmodified adapter.
    rate = 0.0 if self.last_angle is None else (wheel + state.offset - self.last_angle) / 0.01
    kwargs = {
      'speed': v,
      'angle_deg': wheel,
      'rate_deg_s': rate,
      'planned_angle_deg': angle(target_lataccel),
      'plan_t': ModelConstants.T_IDXS,
      'plan_accel': future_plan.acceleration.y,
      'active': True,
      'driver_pressed': False,
      'driver_torque': 0.0,
      'plan_age': 0.0,
      'dt': 0.01,
    }
    target, _ = self.pre.update(command=0.0, target_accel=target_lataccel, **kwargs)
    command = super().update(target, current_lataccel, state, future_plan)
    kwargs['planned_angle_deg'] = angle(target)
    _, command = self.ret.update(command=command, target_accel=target, **kwargs)
    return command


def baseline(meta, cfg):
  return Base(meta, cfg)


def preview(meta, cfg):
  return Controller(meta, cfg, preview=True)


def preview_full(meta, cfg):
  return Controller(meta, cfg, preview=True, preview_gain=1.0)


def release(meta, cfg):
  return Controller(meta, cfg, return_mode='release')


def active(meta, cfg):
  return Controller(meta, cfg, return_mode='active')


def combined(meta, cfg):
  return Controller(meta, cfg, preview=True, return_mode='active')


def inverse(meta, cfg):
  c = Controller(meta, cfg, preview=True)
  c.pre.predictor = "inverse"
  return c

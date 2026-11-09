"""Real V0/NNLC adapter with logged pitch, tuning, delay and safety slew feedback."""
import math
from types import SimpleNamespace as NS
from tools.odyssey_sim.closedloop.controller import Controller as Base


class Controller(Base):
  def update(self, target_lataccel, current_lataccel, state, future_plan=None):
    v = max(float(state.v_ego), .1)
    lp = NS(roll=float(state.roll), angleOffsetDeg=float(state.offset))
    self.vm.update_params(max(float(state.stiffness), .1), max(float(state.ratio), .1))
    angle = math.degrees(self.vm.get_steer_from_curvature(-current_lataccel/v**2, v, lp.roll))+lp.angleOffsetDeg
    rate = 0. if self.last_angle is None else (angle-self.last_angle)/.01
    self.last_angle = angle
    cs = NS(vEgo=v, aEgo=float(state.a_ego), steeringAngleDeg=round(angle*10)/10, steeringRateDeg=rate,
            steeringPressed=False, steeringRateLimited=False)
    self.measured_lataccel = -self.vm.calc_curvature(math.radians(cs.steeringAngleDeg-lp.angleOffsetDeg), v, lp.roll)*v**2
    ext = self.lac.extension
    ext.update_model_v2(future_plan)
    ext.update_lateral_lag(state.delay)
    self.lac.update_torque_parameters(state.factor, state.torque_offset, state.friction*self.cfg.friction)
    ext.update_limits()
    pose = NS(orientation=NS(pitch=float(state.pitch)))
    torque, _, _ = self.lac.update(True, cs, self.vm, lp, bool(state.limited), target_lataccel/v**2, pose, False, state.delay)
    torque = self.odyssey.apply(float(torque), True, cs, self.vm, lp, target_lataccel/v**2, 1.0)
    return -float(torque)


def candidate_v2(meta, cfg):
  """Offline 26-input model adapter from its author's evaluated context contract."""
  from tools.odyssey_sim.nnlc_train.closedloop import CandidateModel
  c = Controller(meta, cfg)
  ext = c.lac.extension
  candidate = CandidateModel(ext)
  original = ext.update_neural_network_feedforward

  def update(cs, params, pose):
    candidate.update(cs)
    return original(cs, params, pose)

  ext.model = candidate
  ext.update_neural_network_feedforward = update
  return c

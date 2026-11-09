"""Offline Odyssey plant, normalized CAN-sign torque (+left), SI units.

load_params(path) loads schema v1 JSON; new_state(params=...) optionally installs
it per state. step(state, torque_cmd, v_ego, dt) returns a NEW dict (no mutation).
State keys: lat_accel (m/s², physical response), steering_accel (m/s² wheel-angle
proxy), motor_cmd (signed normalized effective motor torque), angle_deg (Honda
sign), angle_rate_deg_s, motor_torque (unsigned CAN units). Optional roll (rad),
bias_accel (m/s²), driver_torque (normalized equivalent external torque). Outputs
exclude observation noise. Supply roll explicitly; default is flat road. No plan,
lane feedback, driver-assist model or sensor noise is hidden in this function.
All dynamics use stable exponential substeps <=10 ms. Intended v>=5 m/s; this
is NOT a standstill/parking or safety model. Params live in state['_params'].
"""

import json
import math
from pathlib import Path

DEFAULT_PATH = Path('/private/tmp/claude-501/-Users-williamgrant-Documents-My-vault/ae63c39a-5411-41d2-abc4-99cb7e2003e6/scratchpad/plant_model.json')


def load_params(path=DEFAULT_PATH):
  return json.loads(Path(path).read_text())


def new_state(params=None, **initial):
  state = {
    'lat_accel': 0.0,
    'steering_accel': 0.0,
    'motor_cmd': 0.0,
    'angle_deg': 0.0,
    'angle_rate_deg_s': 0.0,
    'motor_torque': 0.0,
    'roll': 0.0,
    'bias_accel': 0.0,
    'driver_torque': 0.0,
  }
  state['_params'] = load_params() if params is None else params
  state.update(initial)
  return state


def interp(v, x, y):
  if v <= x[0]:
    return y[0]
  for i in range(1, len(x)):
    if v <= x[i]:
      w = (v - x[i - 1]) / (x[i] - x[i - 1])
      return y[i - 1] * (1 - w) + y[i] * w
  return y[-1]


def step(state, torque_cmd, v_ego, dt):
  """Return plant state after dt seconds; torque_cmd clipped to [-1,1]."""
  if dt <= 0 or not math.isfinite(dt) or not math.isfinite(torque_cmd) or not math.isfinite(v_ego):
    raise ValueError('finite torque/speed and positive dt required')
  s = dict(state)
  p = s.get('_params')
  if p is None:
    p = load_params()
    s['_params'] = p
  v = max(v_ego, 5.0)
  gain = interp(v, p['speed_mps'], p['gain'])
  tau = interp(v, p['speed_mps'], p['tau_s'])
  friction = interp(v, p['speed_mps'], p['friction_accel'])
  cmd = min(max(torque_cmd, -1.0), 1.0)
  motor = math.copysign(max(abs(cmd) - p['deadband'], 0.0), cmd)
  h = dt / max(1, math.ceil(dt / 0.01))
  old_angle = s.get('angle_deg', 0.0)
  for _ in range(max(1, math.ceil(dt / 0.01))):
    s['motor_cmd'] = s.get('motor_cmd', 0.0) + (motor - s.get('motor_cmd', 0.0)) * (-math.expm1(-h / p['motor_tau_s']))
    target = gain * (s['motor_cmd'] + s.get('driver_torque', 0.0)) + p['roll_gain'] * s.get('roll', 0.0) + p['bias_accel'] + s.get('bias_accel', 0.0)
    a = s.get('steering_accel', s.get('lat_accel', 0.0))
    push = target - a
    a += math.copysign(max(abs(push) - friction, 0.0), push) * (-math.expm1(-h / tau))
    s['steering_accel'] = a
    s['lat_accel'] = s.get('lat_accel', 0.0) + (a - s.get('lat_accel', 0.0)) * (-math.expm1(-h / p['vehicle_tau_s']))
  s['angle_deg'] = -math.degrees(s['steering_accel'] * p['steer_ratio'] * p['wheelbase_m'] * (1 + p.get('understeer_coefficient', 0.0) * v * v) / (v * v))
  s['angle_rate_deg_s'] = (s['angle_deg'] - old_angle) / dt
  s['motor_torque'] = p.get('motor_baseline', 0.0) + max(abs(s['motor_cmd']) - (p.get('motor_deadband', p['deadband']) - p['deadband']), 0.0) * p['motor_gain']
  return s

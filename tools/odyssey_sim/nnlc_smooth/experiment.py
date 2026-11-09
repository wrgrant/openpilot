"""Deterministic closed-loop controller search, tuning a7 only; immutable raw trials."""

import argparse
import json
import os
from dataclasses import dataclass, asdict
from types import SimpleNamespace as NS
import numpy as np
from tools.odyssey_sim.calib.evaluate import records, pieces
from tools.odyssey_sim.calib.controller import Controller
from tools.odyssey_sim.calib.plant import Plant
from tools.odyssey_sim.closedloop.controller import Config, model_from_array
from tools.odyssey_sim.replay.harness import metadata
from tools.odyssey_sim.nnlc_train import SCRATCH as S, MODEL_DIR
from openpilot.sunnypilot.selfdrive.controls.lib.nnlc.safe_model import SafeNNTorqueModel

OUT = S / 'nnlc_smooth'


@dataclass
class Trial:
  model: str = 'v1'
  error_gain: float = 1.0
  friction: float = 1.0
  ff_tau: float = 0.0
  jerk_scale: float = 1.0
  slew: float = 100.0
  error_tau: float = 0.0
  friction_tau: float = 0.0


def install(ext, trial):
  if trial.model == 'v2':
    from tools.odyssey_sim.nnlc_train.closedloop import CandidateModel

    candidate = CandidateModel(ext)
    original_update = ext.update_neural_network_feedforward

    def update(cs, params, pose):
      candidate.update(cs)
      return original_update(cs, params, pose)

    ext.model = candidate
    ext.update_neural_network_feedforward = update
  elif trial.model != 'stock':
    ext.model = SafeNNTorqueModel(MODEL_DIR / f'HONDA_ODYSSEY_WILLIAM_{trial.model.upper()}.json', MODEL_DIR / 'HONDA_ODYSSEY.json')
  original_eval = ext.model.evaluate

  def evaluate(x):
    x = list(x)
    if len(x) > 2:
      x[2] *= trial.jerk_scale
    return original_eval(x)

  ext.model.evaluate = evaluate
  original_output = ext.update_output_torque
  last_ff = None
  last_error = None
  last_friction = None

  def output(cs):
    nonlocal last_ff, last_error, last_friction
    ext._pid_log.error *= trial.error_gain
    if trial.friction != 1 and ext.model.friction_override:
      # Friction term is in error. Remove its separately identified contribution.
      from opendbc.car.lateral import get_friction, FRICTION_THRESHOLD

      fi = ext.update_friction_input(ext._setpoint, ext._measurement)
      ext._pid_log.error -= trial.error_gain * (1 - trial.friction) * get_friction(fi, ext._lateral_accel_deadzone, FRICTION_THRESHOLD, ext.torque_params)
    if trial.friction_tau and ext.model.friction_override:
      from opendbc.car.lateral import get_friction, FRICTION_THRESHOLD

      fi = ext.update_friction_input(ext._setpoint, ext._measurement)
      f = trial.error_gain * trial.friction * get_friction(fi, ext._lateral_accel_deadzone, FRICTION_THRESHOLD, ext.torque_params)
      last_friction = f if last_friction is None else last_friction + 0.01 / (trial.friction_tau + 0.01) * (f - last_friction)
      ext._pid_log.error += last_friction - f
    if trial.error_tau:
      e = ext._pid_log.error
      last_error = e if last_error is None else last_error + 0.01 / (trial.error_tau + 0.01) * (e - last_error)
      ext._pid_log.error = last_error
    alpha = 1 if trial.ff_tau == 0 else 0.01 / (trial.ff_tau + 0.01)
    last_ff = ext._ff if last_ff is None else last_ff + alpha * (ext._ff - last_ff)
    ext._ff = last_ff
    original_output(cs)

  ext.update_output_torque = output


def factory(meta, cfg):
  trial = Trial(**json.loads(os.environ.get('NNLC_SMOOTH_TRIAL', '{}')))
  c = Controller(meta, Config(nnlc=True))
  install(c.lac.extension, trial)
  return c


def rollout(w, meta, trial, plant_name):
  c = Controller(meta, Config(nnlc=True))
  install(c.lac.extension, trial)
  p = Plant()
  if plant_name == 'public':
    # Public gain/friction/lag sensitivity; retain calibrated body stage and slew.
    from tools.odyssey_sim.closedloop.worker_plant import load_params

    pp = load_params()['public_model']
    # Schema-v1 units differ; handled by alternate plant adapter below.
    from tools.odyssey_sim.closedloop.worker_plant import FittedPlant

    params = load_params()
    params.update(pp)
    FittedPlant.params = params
    alt = FittedPlant()
    alt.reset(float(w['actual'][0]))
  p.reset(w['actual'][0], w['body'][0], -w['steerCommand'][0])
  ys, us, applied = [], [], []
  prev = p.u
  for i in range(len(w['t'])):
    state = NS(v_ego=w['vEgo'][i], a_ego=w['aEgo'][i], roll=w['roll'][i], offset=w['angleOffset'][i], ratio=w['steerRatio'][i], stiffness=w['stiffness'][i])
    for name in ('delay', 'factor', 'friction', 'torque_offset', 'pitch'):
      setattr(state, name, float(w[name][i]))
    state.limited = abs(p.u - prev) > 0.01
    model = model_from_array(w['plans'][i])
    for _ in range(10):
      u = c.update(w['des'][i], p.y, state, model)
      u = float(np.clip(u, prev - trial.slew * 0.01, prev + trial.slew * 0.01))
      prev = u
      p.step(u, state.v_ego, state.roll)
      if plant_name == 'public':
        p.y = alt.step(p.u, state.v_ego, state.a_ego, state.roll, 0.01)
      us.append(u)
      applied.append(p.u)
      ys.append(c.measured_lataccel)
  y, u, applied = map(np.asarray, (ys, us, applied))
  target = np.repeat(w['des'], 10)
  mask = np.arange(len(y)) >= 1000
  err = y - target
  rate = np.diff(y) / 0.01
  dy = np.diff(u)
  # Transparent provisional EPS proxy, replaced/supplemented by perception worker.
  filtered = np.empty_like(u)
  filtered[0] = u[0]
  for i in range(1, len(u)):
    filtered[i] = filtered[i - 1] + 0.01 / (0.87 + 0.01) * (applied[i] - filtered[i - 1])
  m = {
    'cost': float(np.mean(err[mask] ** 2) * 100),
    'jerk': float(np.mean(rate[999:] ** 2)),
    'flips': float(np.sum((u[1000:-1] * u[1001:] < 0) & (abs(dy[1000:]) > 0.01)) / ((len(u) - 1000) * 0.01)),
    'eps_proxy': float(np.sqrt(np.mean((np.diff(filtered)[999:] / 0.01) ** 2))),
    'command_rate': float(np.sqrt(np.mean((dy[999:] / 0.01) ** 2))),
    'saturation': float(np.mean(abs(u[mask]) >= 0.99)),
    'n': int(mask.sum()),
  }
  for phase, phase_mask in [('turn_in', np.abs(np.repeat(np.gradient(w['des'], 0.1), 10)) > 0.1), ('mid', abs(target) > 0.3), ('straight', abs(target) < 0.1)]:
    z = mask & phase_mask
    m[phase] = float(np.mean(err[z] ** 2) * 100) if z.any() else None
  return m, {
    'y': y,
    'u': u,
    'applied': applied,
    'target': target,
    'v': np.repeat(w['vEgo'], 10),
    'roll': np.repeat(w['roll'], 10),
    'ratio': np.repeat(w['steerRatio'], 10),
    'stiffness': np.repeat(w['stiffness'], 10),
  }


def trials():
  yield Trial(model='stock')
  yield Trial()
  for gain in (0.15, 0.3, 0.5, 0.7, 1.0, 1.4):
    for friction in (0.0, 0.5, 1.0):
      yield Trial(error_gain=gain, friction=friction)
  for tau in (0.03, 0.08, 0.15, 0.3):
    yield Trial(ff_tau=tau, error_gain=0.5, friction=0)
  for jerk in (0.0, 0.25, 0.5):
    yield Trial(error_gain=0.5, friction=0, jerk_scale=jerk)
  for slew in (0.3, 0.6, 1.0, 1.5):
    yield Trial(error_gain=0.5, friction=0, slew=slew)
  for tau in (0.02, 0.05, 0.1, 0.2, 0.4):
    yield Trial(error_tau=tau)
    yield Trial(friction_tau=tau)
  for slew in (0.2, 0.3, 0.5, 0.8, 1.0, 1.5, 2.0, 3.0):
    yield Trial(slew=slew)
  for gain in (1.4, 2.0, 3.0):
    for tau in (0.05, 0.15):
      yield Trial(error_gain=gain, error_tau=tau)
  for gain in (1.0, 1.4):
    for tau in (0.02, 0.05, 0.1, 0.2):
      yield Trial(error_gain=gain, ff_tau=tau)
  rng = np.random.default_rng(20261109)
  for _ in range(24):
    yield Trial(
      error_gain=float(rng.uniform(0.8, 1.8)), friction=float(rng.uniform(0.6, 1)), error_tau=float(rng.uniform(0, 0.1)), ff_tau=float(rng.uniform(0, 0.08))
    )

  yield Trial(model='v2')


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('--test', action='store_true')
  ap.add_argument('--limit', type=int, default=999)
  ap.add_argument('--start', type=int, default=0)
  args = ap.parse_args()
  OUT.mkdir(exist_ok=True)
  os.environ['OPENPILOT_PREFIX'] = 'simnnlcsmooth'
  data = list(records())
  rows = json.loads((OUT / 'generations.json').read_text()) if args.start else []
  candidates = list(trials())[: args.limit]
  if args.test:
    candidates = [Trial(model='stock'), Trial()] + [Trial(**x) for x in json.loads((OUT / 'selected.json').read_text())] + [Trial(model='v2')]
  for k, trial in enumerate(candidates):
    if k < args.start:
      continue
    for route, d in data:
      if not (route.startswith(('000001aa', '000001ab')) if args.test else route.startswith('000001a7')):
        continue
      rs = list(pieces(d)) or list(pieces(d, 30))
      for j, r in enumerate(rs):
        w = {key: value[r] for key, value in d.items()}
        for plant in ('william', 'public'):
          m, trace = rollout(w, metadata(route), trial, plant)
          row = dict(generation=k, trial=asdict(trial), route=route, window=j, plant=plant, **m)
          rows.append(row)
          np.savez_compressed(OUT / f'{"test" if args.test else "selection"}_{k}_{route[:8]}_{j}_{plant}.npz', **trace)
    (OUT / ('test.json' if args.test else 'generations.json')).write_text(json.dumps(rows, indent=2))
    print(
      k, asdict(trial), {p: round(np.mean([r['cost'] for r in rows if r['generation'] == k and r['plant'] == p]), 4) for p in ('william', 'public')}, flush=True
    )


def runtime_factory(meta, cfg):
  """Exercise actual override construction, including JSON shaping metadata."""
  from openpilot.common.params import Params

  p = Params()
  p.put('NnlcModelOverride', os.environ.get('NNLC_SMOOTH_MODEL', 'HONDA_ODYSSEY_WILLIAM_S3.json'), block=True)
  try:
    return Controller(meta, Config(nnlc=True))
  finally:
    p.remove('NnlcModelOverride')


if __name__ == '__main__':
  main()

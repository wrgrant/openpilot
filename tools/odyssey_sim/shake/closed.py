"""Run real V0 + NNLC/response shaper + OdysseySteer against frozen plant.

No replayed vehicle state or command feeds the closed loop. Logged model plans,
calibration and desired curvature are exogenous. Params are scratch-isolated.
"""
import json
from types import SimpleNamespace as NS
from pathlib import Path
import numpy as np
from scipy import signal
from tools.odyssey_sim.shake.extract import SCRATCH
from tools.odyssey_sim.shake.analyze import OUT, windows, band
from tools.odyssey_sim.shake.physics import advance
import openpilot.common.params as params_module

OriginalParams = params_module.Params


class ScratchParams(OriginalParams):
  def __init__(self, d=''):
    super().__init__(str(SCRATCH/'params'))

  def get(self, key, *args, encoding=None, **kwargs):
    value = super().get(key, *args, **kwargs)
    return value.decode(encoding) if encoding and isinstance(value, bytes) else value


params_module.Params = ScratchParams
from tools.odyssey_sim.closedloop.controller import Controller, Config, model_from_array
from tools.odyssey_sim.replay.harness import metadata
from openpilot.sunnypilot.selfdrive.controls.lib.nnlc.safe_model import SafeNNTorqueModel
from openpilot.sunnypilot.selfdrive.controls.lib.nnlc.nnlc import NnlcResponseShaper


def stats(command, angle, body):
  u = command[500:-100]
  crossing = np.flatnonzero(u[1:]*u[:-1] < 0)+1
  later = crossing[crossing+10 < len(u)]+10
  f, psd = signal.welch(band(body)[500:-100], fs=100, nperseg=1024)
  mask = (f >= .3) & (f <= 3)
  return {'crossings_s': float(len(crossing)/(len(u)*.01)),
          'after_crossing': float(np.median(abs(u[later]))) if len(later) else 0.,
          'command_rms': float(np.std(band(command)[500:-100])),
          'wheel_rms_deg': float(np.std(band(angle)[500:-100])),
          'body_rms': float(np.std(band(body)[500:-100])),
          'peak_per_second': float(f[mask][np.argmax(psd[mask])])}


def main():
  p = np.array(json.loads((OUT/'fit.json').read_text())['parameters'])
  rows = []
  for name in ('000001ad', '000001ae'):
    meta = metadata(name)
    d = dict(np.load(SCRATCH/f'{name}.npz'))
    for w in windows(name):
      c = Controller(meta, Config(nnlc=True))
      ext = c.lac.extension
      stock = Path.cwd()/'openpilot/sunnypilot/neural_network_data/neural_network_lateral_control/HONDA_ODYSSEY.json'
      if w['model']:
        path = Path.cwd()/f'openpilot/sunnypilot/selfdrive/controls/lib/nnlc/odyssey_models/HONDA_ODYSSEY_WILLIAM_S{3 if w["model"] == 1 else 5}.json'
        ext.model = SafeNNTorqueModel(path, stock)
        ext.response_shaper = NnlcResponseShaper.load(path)
      ext.model_ab = None
      c.lac.reset()
      state = w['initial'].copy()
      us, angles, bodies = [], [], []
      ix = np.clip(np.searchsorted(d['t'], w['t']), 0, len(d['t'])-1)
      for i, k in enumerate(ix):
        v = w['v'][i]
        vehicle, tuning = d['vehicle'][k], d['tuning'][k]
        lp = NS(roll=float(vehicle[0]), angleOffsetDeg=float(vehicle[1]))
        c.vm.update_params(max(float(vehicle[3]), .1), max(float(vehicle[2]), .1))
        cs = NS(vEgo=v, aEgo=0., steeringAngleDeg=round(-np.degrees(state[0])*10)/10,
                steeringRateDeg=-np.degrees(state[1]), steeringPressed=False, steeringRateLimited=False, leftBlinker=False, rightBlinker=False)
        ext.update_model_v2(model_from_array(d['plans'][k].reshape(3, -1)))
        delay = float(d['delay'][k, 0])
        ext.update_lateral_lag(delay)
        ext.update_limits()
        c.lac.update_torque_parameters(*map(float, tuning))
        pose = NS(orientation=NS(pitch=float(d['orientation'][k, 1])))
        desired = w['des_curv'][i]
        limited = len(state) > 9 and len(us) > 0 and abs(state[9]-us[-1]) > .01
        torque, _, _ = c.lac.update(True, cs, c.vm, lp, limited, desired, pose, False, delay)
        torque = c.odyssey.apply(float(torque), True, cs, c.vm, lp, desired, 1.)
        command = -float(torque)
        y, state = advance(np.array([command]), np.array([v]), p, state)
        us.append(command)
        angles.append(np.degrees(y[0, 0]))
        bodies.append(y[0, 1])
      road = stats(w['u'], -w['angle'], w['truth'][:, 1])
      sim = stats(np.array(us), np.array(angles), np.array(bodies))
      ratios = {key: sim[key]/road[key] if road[key] > 0 else None for key in road if key != 'peak_per_second'}
      row = {'route': name, 'start': float(w['t'][0]), 'model': w['model'], 'lane': w['lane'], 'speed': w['speed'],
             'road': road, 'sim': sim, 'ratios': ratios,
             'peak_error': sim['peak_per_second']-road['peak_per_second']}
      rows.append(row)
      print(name, w['model'], w['speed'], ratios, flush=True)
  summary = []
  for model in range(3):
    group = [r for r in rows if r['model'] == model]
    ratios = {key: float(np.median([r['ratios'][key] for r in group if r['ratios'][key] is not None])) for key in rows[0]['ratios']}
    peak_error = float(np.median([abs(r['peak_error']) for r in group]))
    summary.append({'model': model, 'windows': len(group), 'ratios': ratios, 'median_abs_peak_error': peak_error,
                    'passed': all(.75 <= x <= 1.25 for x in ratios.values()) and (model != 2 or peak_error <= .3)})
  result = {'summary': summary, 'rows': rows, 'passed': all(r['passed'] for r in summary),
            'ranking_test': 'Not authorized: plant gate failed; descriptive metrics only.'}
  (OUT/'gate2.json').write_text(json.dumps(result, indent=2))
  print(summary, flush=True)


if __name__ == '__main__':
  main()

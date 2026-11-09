"""Offline camera-conditioned lane reconstruction for controlled simulator experiments.

Road turning is reconstructed from independently observed lane heading and yaw.
It is intentionally labelled reconstructed, never a surveyed independent road.
"""

import json
import numpy as np
from tools.odyssey_sim.calib.evaluate import records as base_records
from tools.odyssey_sim.calib3.extract import OUT


def records(forcing="fusion"):
  selection = json.loads((OUT / 'transport.json').read_text())['selection']
  for route, d in base_records():
    z = np.load(OUT / f'{route[:8]}.npz')
    ix = np.clip(np.searchsorted(z['t'], d['t'], side='right') - 1, 0, len(z['t']) - 1)
    variant = 1 if forcing == 'fusion' else selection['variant']
    e, h, curvature = z['geometry'][ix, variant].T
    vel = d['vEgo'] * np.tan(h) * selection['gain'] - selection['vy_gain'] * z['velocity'][ix, 1, 0]
    vel = np.interp(d['t'] + selection['shift'], d['t'], vel)
    vel = np.convolve(np.pad(vel, (5, 5), mode='edge'), np.ones(11) / 11, mode='valid')
    if forcing == 'fusion':
      coef = np.asarray(json.loads((OUT / 'fusion.json').read_text())['coefficients_spatial_transport_model_vy'])
      vh = d['vEgo'] * np.tan(h)
      vy = z['velocity'][ix, 1, 0]
      vh = np.convolve(np.pad(vh, (5, 5), mode='edge'), np.ones(11) / 11, mode='valid')
      vy = np.convolve(np.pad(vy, (5, 5), mode='edge'), np.ones(11) / 11, mode='valid')
      delta = coef[0] * (d['vEgo'] ** 2 * curvature - d['body']) + coef[1] * np.gradient(vh, 0.1) + coef[2] * np.gradient(vy, 0.1)
      d['road'] = d['body'] + delta
    else:
      d['road'] = d['body'] + np.gradient(vel, 0.1)
    d['offset'] = e
    yield route, d


def consistent_future(model, target_delta):
  """Local-plan perturbation: acceleration change fades at a 1.5 s horizon.

  Logged roll/pitch are exogenous. This is an explicit surrogate, not modeld.
  """
  from types import SimpleNamespace as NS
  from openpilot.selfdrive.modeld.constants import ModelConstants

  delta = target_delta * np.exp(-np.asarray(ModelConstants.T_IDXS) / 1.5)
  return NS(acceleration=NS(y=np.asarray(model.acceleration.y) + delta), orientation=model.orientation)

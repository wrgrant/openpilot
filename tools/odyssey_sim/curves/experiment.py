"""Curve-window plant-v2 rejection tests and fixed-motion controller replay.

All variants are fixed hypotheses, not a held-out parameter search. Offline
motion/plan cache is 10 Hz, real LaC executes at 100 Hz, same as calib evaluator.
"""

import json
import math
from types import SimpleNamespace as NS
import numpy as np
from tools.odyssey_sim.calib.evaluate import records
from tools.odyssey_sim.calib.plant import Plant
from tools.odyssey_sim.calib.identify import S
from tools.odyssey_sim.closedloop.controller import Config, model_from_array
from tools.odyssey_sim.curves import controller
from tools.odyssey_sim.replay.harness import metadata
from tools.odyssey_sim.perception.metric import perceptible_roughness
from tools.odyssey_sim.curves.characterize import KEYS, summary

OUT = S / 'curves'
VARIANTS = ['baseline', 'preview', 'preview_full', 'release', 'active', 'combined']


def run(c, w, closed):
  plant = Plant()
  plant.reset(w['actual'][0], w['body'][0], -w['steerCommand'][0])
  ys, bs, us, angles, targets = [], [], [], [], []
  for i in range(len(w['t'])):
    state = NS(
      v_ego=w['vEgo'][i],
      a_ego=w['aEgo'][i],
      roll=w['roll'][i],
      offset=w['angleOffset'][i],
      ratio=w['steerRatio'][i],
      stiffness=w['stiffness'][i],
      limited=False,
    )
    for k in ['delay', 'factor', 'friction', 'torque_offset', 'pitch']:
      setattr(state, k, float(w[k][i]))
    model = model_from_array(w['plans'][i])
    for _ in range(10):
      y = plant.y if closed else w['actual'][i]
      u = c.update(w['des'][i], y, state, model)
      plant.step(u, state.v_ego, state.roll)
      ys.append(plant.y if closed else y)
      bs.append(plant.body if closed else w['body'][i])
      us.append(u)
      angles.append(math.degrees(c.vm.get_steer_from_curvature(-ys[-1] / state.v_ego**2, state.v_ego, state.roll)))
      targets.append(w['des'][i] + (c.pre.correction if hasattr(c, 'pre') else 0.0))
  return {k: np.asarray(v) for k, v in zip(['y', 'body', 'u', 'angle', 'target'], [ys, bs, us, angles, targets], strict=True)}


def main():
  ev = json.loads((OUT / 'characterization.json').read_text())['events']
  rows = []
  traces = {}
  for key, d in records():
    if key[:8] not in KEYS:
      continue
    meta = metadata(key)
    # One 14 s window per event, 4 s context before detected entry. No matching
    # or selection against candidate response; all variants use identical frames.
    for j, event in enumerate(ev[key[:8]]['rows']):
      ix = np.flatnonzero((d['t'] >= event['start_t'] - 4) & (d['t'] < event['start_t'] + 10))
      if len(ix) < 130 or not d['ok'][ix].all() or np.max(np.diff(d['t'][ix])) > 0.16:
        continue
      w = {k: v[ix] for k, v in d.items()}
      ref = np.repeat(w['des'], 10)
      for mode in ['plant', 'replay']:
        outputs = {}
        for variant in VARIANTS:
          c = getattr(controller, variant)(meta, Config(nnlc=True))
          z = run(c, w, mode == 'plant')
          outputs[variant] = z
          score = slice(400, None)
          rough = perceptible_roughness(dt=0.01, angle_deg=z['angle'], lat_accel=z['body'], command=z['u'])
          base = outputs['baseline']
          change = abs(z['u'] - base['u']) > 0.02
          phase = np.repeat(np.r_[0, np.diff(abs(w['des']))], 10)
          r = {
            'route': key[:8],
            'event': j,
            'start_t': float(w['t'][0]),
            'mode': mode,
            'variant': variant,
            'mse': float(np.mean((z['y'][score] - ref[score]) ** 2)),
            'saturation_pct': float(100 * np.mean(abs(z['u'][score]) >= 0.99)),
            'roughness': rough['roughness'],
            'changed_pct': float(100 * np.mean(change)),
            'command_delta_peak': float(np.max(abs(z['u'] - base['u']))),
            'entry_command_delta': float(np.mean((z['u'] - base['u'])[phase > 0.002])) if np.any(phase > 0.002) else 0.0,
            'exit_command_delta': float(np.mean((z['u'] - base['u'])[phase < -0.002])) if np.any(phase < -0.002) else 0.0,
            'target_delta_peak': float(np.max(abs(z['target'] - ref))),
          }
          rows.append(r)
          if j == 0:
            traces[f'{key[:8]}_{mode}_{variant}'] = np.array([z[k] for k in ['y', 'body', 'u', 'angle', 'target']])
      print(key[:8], j, flush=True)
      (OUT / 'experiments.json').write_text(json.dumps(rows, indent=2))
  np.savez_compressed(OUT / 'traces.npz', **traces)
  for mode in ['plant', 'replay']:
    for v in VARIANTS:
      rr = [r for r in rows if r['mode'] == mode and r['variant'] == v]
      print(mode, v, {k: summary([r[k] for r in rr]) for k in ['mse', 'roughness', 'changed_pct', 'command_delta_peak']}, flush=True)


if __name__ == '__main__':
  main()

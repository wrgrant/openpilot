"""Score continuous unquantized plant motion, never differentiated CAN steps."""

import json
import math
from dataclasses import replace
import numpy as np
from opendbc.car.vehicle_model import VehicleModel
from tools.odyssey_sim.calib.plant import Plant
from tools.odyssey_sim.closedloop.worker_plant import FittedPlant, load_params
from tools.odyssey_sim.replay.harness import metadata
from tools.odyssey_sim.perception.metric import perceptible_roughness, sluggishness, Thresholds
from tools.odyssey_sim.nnlc_smooth.experiment import OUT


def score_trace(path, route, plant):
  z = np.load(path)
  v = z['v']
  roll = z['roll']
  u = z['u']
  p = Plant()
  p.reset(z['y'][0], u=z['applied'][0])
  alt = None
  if plant == 'public':
    params = load_params()
    params.update(params['public_model'])
    FittedPlant.params = params
    alt = FittedPlant()
    alt.reset(z['y'][0])
  vm = VehicleModel(metadata(route)['carParams'].as_reader())
  angles = []
  body = []
  for i in range(len(u)):
    if 'ratio' in z:
      vm.update_params(max(float(z['stiffness'][i]), 0.1), max(float(z['ratio'][i]), 0.1))
    p.step(u[i], v[i], roll[i])
    if alt is not None:
      p.y = alt.step(p.u, v[i], 0.0, roll[i], 0.01)
    angles.append(math.degrees(vm.get_steer_from_curvature(-p.y / max(v[i], 0.1) ** 2, v[i], roll[i])))
    body.append(p.body)
  angles = np.asarray(angles)[1000:]
  body = np.asarray(body)[1000:]
  result = perceptible_roughness(dt=0.01, angle_deg=angles, angle_rate_deg_s=np.gradient(angles, 0.01), lat_accel=body, command=u[1000:])
  # Plan-normalized residual asks whether roughness is simply required turning.
  desired_angle = np.array(
    [math.degrees(vm.get_steer_from_curvature(-a / max(speed, 0.1) ** 2, speed, r)) for a, speed, r in zip(z['target'], v, roll, strict=True)]
  )[1000:]
  residual = perceptible_roughness(dt=0.01, angle_deg=angles - desired_angle, lat_accel=body - z['target'][1000:])
  sensitivity = {}
  for scale in (0.25, 0.5, 1.0, 2.0):
    th = Thresholds()
    th = replace(th, **{k: v * scale for k, v in vars(th).items() if k != 'wheel_radius_m'})
    sensitivity[str(scale)] = perceptible_roughness(dt=0.01, angle_deg=angles, angle_rate_deg_s=np.gradient(angles, 0.01), lat_accel=body, thresholds=th)[
      'roughness'
    ]
  return {
    'sensitivity': sensitivity,
    'roughness': result['roughness'],
    'roughness_windows': result['scored_windows'],
    'roughness_residual': residual['roughness'],
    'lag': sluggishness(dt=0.01, actual=z['y'][1000:], reference=z['target'][1000:], threshold=0.05),
  }


def main():
  import argparse

  ap = argparse.ArgumentParser()
  ap.add_argument('--test', action='store_true')
  a = ap.parse_args()
  name = 'test' if a.test else 'selection'
  path = OUT / ('test.json' if a.test else 'generations.json')
  rows = json.loads(path.read_text())
  for r in rows:
    trace = OUT / f"{name}_{r['generation']}_{r['route'][:8]}_{r['window']}_{r['plant']}.npz"
    r.update(score_trace(trace, r['route'], r['plant']))
  (OUT / f'{name}_scored.json').write_text(json.dumps(rows, indent=2))
  for gen in sorted({r['generation'] for r in rows}):
    rs = [r for r in rows if r['generation'] == gen]
    print(
      gen,
      rs[0]['trial'],
      {
        p: {k: round(np.mean([r[k] for r in rs if r['plant'] == p]), 4) for k in ('cost', 'flips', 'roughness', 'roughness_residual')}
        for p in ('william', 'public')
      },
      flush=True,
    )


if __name__ == '__main__':
  main()

"""Independent gate-3 diagnostic: no camera/model, no plant retuning."""
import argparse
import json
import numpy as np
from tools.odyssey_sim.calib.identify import load
from tools.odyssey_sim.calib.plant import Plant
from tools.odyssey_sim.vsim.geometry import RelativePose


def audit(route, native=False):
  d = load(route)
  if native:
    from tools.odyssey_sim.vsim.closed_loop import native_data
    raw = native_data(route)
    for k in ('t', 'cmd'):
      d['native_' + k] = raw[k]
  out = []
  for i in range(0, len(d['t']) - 600, 600):
    sl = slice(i, i + 600)
    if not d['ok'][sl].all() or np.max(np.diff(d['t'][sl])) > .16:
      continue
    plant = Plant()
    plant.reset(d['act'][i], d['body'][i], d['cmd'][i])
    pose, oracle = RelativePose(), RelativePose()
    y, a = [], []
    for j in range(i, i + 599):
      dt = (d['t'][j + 1] - d['t'][j]) / 10
      for tick in range(10):
        u = d['cmd'][j]
        if native:
          ix = np.searchsorted(d['native_t'], d['t'][j] + tick * dt, side='right') - 1
          u = d['native_cmd'][ix]
        plant.step(u, d['v'][j], d['roll'][j], dt)
        pose.step(d['v'][j], plant.body / d['v'][j], d['yaw'][j], dt)
        oracle.step(d['v'][j], d['yaw'][j], d['yaw'][j], dt)
      y.append(pose.dy)
      a.append(plant.body - d['body'][j + 1])
    out.append({'route': route, 't0': float(d['t'][i]), 'duration_s': float(d['t'][i + 599] - d['t'][i]),
                'dy_end_m': float(pose.dy), 'dy_rms_m': float(np.sqrt(np.mean(np.square(y)))),
                'heading_end_deg': float(np.rad2deg(pose.dpsi)), 'body_rmse': float(np.sqrt(np.mean(np.square(a)))),
                'oracle_dy_m': float(oracle.dy)})
  return out


def main():
  ap = argparse.ArgumentParser(description=__doc__)
  ap.add_argument('--route', default='000001a3')
  ap.add_argument('--output', required=True)
  ap.add_argument('--native', action='store_true')
  args = ap.parse_args()
  rows = audit(args.route, args.native)
  with open(args.output, 'w') as f:
    json.dump(rows, f, indent=2)
  print(json.dumps(rows, indent=2))


if __name__ == '__main__':
  main()

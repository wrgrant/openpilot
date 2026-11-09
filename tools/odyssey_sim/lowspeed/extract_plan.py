"""Cache current-frame model horizon only, bounded to low-speed-containing segments."""

import numpy as np
from pathlib import Path
from openpilot.tools.lib.logreader import LogReader
from tools.odyssey_sim.plant.extract import SCRATCH as S

O = S / 'lowspeed'


def main():
  O.mkdir(exist_ok=True)
  for r in ['1a7', '1aa', '1ab']:
    dest = O / f'plan_{r}.npz'
    if dest.exists():
      continue
    d = np.load(S / f'plant_data/00000{r}.npz')
    segs = np.unique(d['seg'][(d['v'] > 0.5) & (d['v'] < 8) & (d['active'] > 0)])
    route = next(Path.home().joinpath('comma-logs').glob(f'*00000{r}*'))
    rows = []
    for seg in segs:
      p = route / (route.name.split('_', 1)[1] + '--' + str(int(seg))) / 'rlog.zst'
      for m in LogReader(str(p)):
        if m.which() == 'modelV2':
          q = m.modelV2
          if len(q.orientationRate.z) == 33 and len(q.velocity.x) == 33:
            rows.append([m.logMonoTime / 1e9, *q.orientationRate.z, *q.velocity.x])
    a = np.asarray(rows)
    np.savez_compressed(dest, t=a[:, 0], yaw=a[:, 1:34], speed=a[:, 34:], horizon=np.asarray(q.orientationRate.t))
    print(r, len(a), flush=True)


if __name__ == '__main__':
  main()
